use geometry::bxdf::{BxDF, BXDF, Omega};
use math::prob::Prob;
use math::{assert_lt, hcm};
use radiometry::color::Color;
use shape::Interaction;

/// bsdf.rs
///
/// Models the scattering of rays at a surface intersection, in the global coordinate space.
///
pub struct BSDF<'a> {
    frame: hcm::Mat3,
    bxdfs: &'a [BXDF<'a>],
}

impl<'a> BSDF<'a> {
    /// Creates a new BSDF with geometry information and material.
    pub fn new_frame(isect: &Interaction) -> Self {
        let normal = isect.normal.hat();
        let bitangent = isect.normal.cross(isect.tangent()).hat();
        let tangent = bitangent.cross(normal);
        assert!(normal.dot(bitangent).abs() < 1e-4);
        assert!(normal.dot(tangent).abs() < 1e-4);
        assert!(tangent.dot(bitangent).abs() < 1e-4);
        let res = Self {
            frame: hcm::Mat3::from_cols(tangent, bitangent, normal),
            bxdfs: &[],
        };
        assert!(res.has_valid_frame());
        res
    }

    #[allow(dead_code)]
    pub fn frame(&self) -> hcm::Mat3 {
        self.frame
    }

    /// Adds a set of BxDFs to the frame. The BSDF object shouldn't outlive the `bxdfs`.
    pub fn with_bxdfs(self, bxdfs: &'a Vec<BXDF<'a>>) -> Self {
        Self { bxdfs, ..self }
    }

    pub fn eval(&self, wo: hcm::Vec3, wi: hcm::Vec3) -> Color {
        let wi = self.world_to_local(wi);
        let wo = self.world_to_local(wo);
        if wo.z() == 0.0 {
            return Color::black();
        } else {
            self.bxdfs.iter().map(|b| b.eval(wo, wi)).sum()
        }
    }

    pub fn pdf(&self, wo: hcm::Vec3, wi: hcm::Vec3) -> f32 {
        let wi = self.world_to_local(wi);
        let wo = self.world_to_local(wo);
        self.bxdfs.iter().map(|b| b.prob(wo, wi).density()).sum()
    }

    pub fn sample(&self, wo_world: hcm::Vec3, rnd2: (f32, f32)) -> (Color, hcm::Vec3, Prob) {
        // Chooses a bxdf to sample.
        assert_eq!(2.5 as i32, 2);
        assert_lt!(rnd2.0, 1.0);

        let wo = self.world_to_local(wo_world);

        let mut bxdfs = self.bxdfs.iter().collect::<Vec<_>>();
        if bxdfs.is_empty() {
            // No BxDFs are available in this BSDF object.
            return (Color::black(), hcm::Vec3::ZERO, Prob::Mass(0.0));
        }
        // Picks a BxDF from the available diffuse ones, randomly using u.
        //     0         1        2        3        4    ...         n-2       n-1
        // +--------+--------+--------+--------+--------+--------+--------+--------+
        // | <---------- u * n -------+---> |  |
        //                            + chosen +  chosen = floor(u * n)
        //                            |remap|     remapped u = fract(u * n)
        let (u, v) = rnd2;
        let chosen_index = (u * bxdfs.len() as f32) as usize;
        let remapped_u = (u * bxdfs.len() as f32).fract();
        let rnd2 = (v, remapped_u);

        // Isolates the chosen BxDF. Uses this BxDF to sample an incident direction.
        let chosen_bxdf = bxdfs.swap_remove(chosen_index);
        let (bsdf_value, wi, prob) = chosen_bxdf.sample(wo, rnd2);

        // Returns in advance if the sampled bxdf is from a delta-dirac distribution.
        if matches!(prob, Prob::Mass(_)) {
            return (bsdf_value, self.local_to_world(wi), prob);
        }

        // Uses the rest of the bxdfs to compute a tallied bsdf value and an averaged pdf.
        let probs = bxdfs
            .iter()
            .map(|bxdf| bxdf.prob(wo, wi))
            .filter(|pr| pr.is_density());
        let other_pdf_count = probs.clone().count();
        let other_pdf_sum = probs.map(|pr| pr.density()).sum::<f32>();
        let overall_pdf = (prob.density() + other_pdf_sum) / (1 + other_pdf_count) as f32;
        (
            bsdf_value + bxdfs.iter().map(|bxdf| bxdf.eval(wo, wi)).sum(),
            self.local_to_world(wi),
            Prob::Density(overall_pdf),
        )
    }
    pub fn sample_specular(&self, wo_world: hcm::Vec3) -> Option<(Color, hcm::Vec3, Prob)> {
        let wo = self.world_to_local(wo_world);
        for bxdf in self.bxdfs {
            if let BXDF::Specular(spec_bxdf) = bxdf {
                let (f, wi, pr) = spec_bxdf.sample(wo, (0.0, 0.0));
                return Some((f, self.local_to_world(wi), pr));
            }
        }
        return None;
    }
    pub fn world_to_local(&self, world: hcm::Vec3) -> Omega {
        let cols = self.frame.cols;
        assert!(self.has_valid_frame());
        Omega::normalize(cols[0].dot(world), cols[1].dot(world), cols[2].dot(world))
    }

    pub fn local_to_world(&self, local: Omega) -> hcm::Vec3 {
        let cols = self.frame.cols;
        assert!(self.has_valid_frame());
        local.x() * cols[0] + local.y() * cols[1] + local.z() * cols[2]
    }
    pub fn has_valid_frame(&self) -> bool {
        let cols = self.frame.cols;
        let det = cols[0].cross(cols[1]).dot(cols[2]);
        if (det - 1.0).abs() < 1e-4 {
            true
        } else {
            println!("frame = {} {} {}", cols[0], cols[1], cols[2]);
            println!("frame^T frame = {:?}", self.frame * self.frame.transpose());
            println!("det = {}", det);
            false
        }
    }
}

// + Glass: FresnelSpecular / Trowbridge (refl + transmit) / Specular (refl + transmit)
// + Matte: Lambertian / OrenNayar
// - Metal: Trowbridge FresnelConductor
// + Mirror: FresnelNoOp Specular Reflection
// + Plastic: Lambertian + Trowbridge Reflection
// - Substrate: Trowbridge FresnelBlend
// - Translucent: lambertian + trowbridge reflection + trowbridge transmit
//
// fourier:  FourierBSDF
// Uber: specular + lambertian reflection + trowbridge refl

#[cfg(test)]
mod test {
    use super::BSDF;
    use geometry::bxdf::{self, BXDF};
    use math::float::Float;
    use math::hcm;
    use math::prob::Prob;
    use radiometry::color::Color;
    use shape::Interaction;
    #[test]

    fn mf_refl_test() {
        let alpha = geometry::microfacet::MicrofacetDistrib::roughness_to_alpha(0.2);
        let albedo = Color::white();
        let distrib = geometry::microfacet::MicrofacetDistrib::beckmann(alpha, alpha);
        let mf_refl = bxdf::MicrofacetReflection::new(albedo, distrib, bxdf::Fresnel::Nop);

        let bxdfs: Vec<BXDF> = vec![mf_refl.into()];

        let normal = hcm::vec3(-0.6, 0.5, 0.2).hat();
        let (dpdu, dpdv) = hcm::make_coord_system(normal);
        let frame = hcm::Mat3::from_cols(dpdu, dpdv, normal);
        assert!(
            (frame * frame.transpose() - hcm::Mat3::IDENTITY).frobenius_norm_squared() < 1e-6
        );

        let isect = Interaction::rayless(hcm::Point3::new(3.0, 2.5, 2.0), (0.2, 0.8), normal)
            .with_dpdu(dpdu);

        let bsdf = BSDF::new_frame(&isect).with_bxdfs(&bxdfs);
        assert!(
            (frame - bsdf.frame()).frobenius_norm_squared() < 1e-6,
            "frames: {:?} vs {:?}",
            frame,
            bsdf.frame()
        );

        let wo_local = hcm::vec3(0.6, 0.0, 0.8).hat();

        let wo_world = frame * wo_local;
        assert!(wo_world.dot(normal).dist_to(wo_local.z) < 1e-3);
        {
            let wo_local_actual = bsdf.world_to_local(wo_world).0;
            assert!(
                (wo_local - wo_local_actual).norm_squared() < 1e-6,
                "{} vs {}",
                wo_local,
                wo_local_actual
            );
        }
        let (uvec, _du) = math::float::linspace((0.0, 1.0), 10);
        let vvec = uvec.clone();

        let mut colors = vec![];
        for u in uvec.iter() {
            for v in vvec.iter() {
                let (u, v) = (*u, *v);
                let (bsdf_value, wi_world, pr) = bsdf.sample(wo_world, (u, v));
                match pr {
                    Prob::Density(pdf) => {
                        let response =
                            bsdf_value * wi_world.dot(isect.normal).abs() * pdf.weak_recip();
                        println!("response = {}, pr = {}", response, pdf);
                        if pdf > 0.0 {
                            assert!(!response.r.is_nan());
                            colors.push(response);
                        }
                    }
                    Prob::Mass(_) => panic!("refl shouldn't give probability mass!"),
                }
            }
        }

        let mean_color = Color::average(&colors);
        // TODO: maybe add some assertions on the mean color.
        println!("mean color = {}", mean_color);
    }
}
