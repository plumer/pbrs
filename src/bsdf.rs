use geometry::bxdf::{BxDF, Omega};
use math::prob::Prob;
use math::{assert_lt, hcm};
use radiometry::color::Color;
use shape::Interaction;

/// bsdf.rs
///
/// Models the scattering of rays at a surface intersection, in the global coordinate space.
///
pub struct BSDF <'a> {
    frame: hcm::Mat3,
    bxdfs: &'a[Box<dyn BxDF>],
}

impl <'a> BSDF <'a> {
    /// Creates a new BSDF with geometry information and material.
    pub fn new_frame(isect: &Interaction) -> Self {
        let normal = isect.normal.hat();
        let bitangent = isect.normal.cross(isect.tangent()).hat();
        let tangent = bitangent.cross(isect.normal);
        Self {
            frame: hcm::Mat3::from_vectors(tangent, bitangent, normal),
            bxdfs: &[],
        }
    }

    pub fn with_bxdfs(self, bxdfs: &'a Vec<Box<dyn BxDF>>) -> Self {
        Self { bxdfs: bxdfs.as_slice(), ..self }
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
            return (Color::black(), hcm::Vec3::zero(), Prob::Mass(0.0));
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
        (det - 1.0).abs() < 1e-4
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
