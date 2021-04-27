use crate::hcm;
use crate::{ray};

/// Left-handed coordinate system camera: x rightward, y upward, z forward.
pub struct Camera {
    center: hcm::Point3,
    a: hcm::Vec3,
    b: hcm::Vec3,
    c: hcm::Vec3,

    // Film image resolution.
    width: u32,
    height: u32,

    orientation: [hcm::Vec3; 3],
}

impl Camera {
    pub fn new(resolution: (u32, u32), fov_y: hcm::Radian) -> Camera {
        let (width, height) = resolution;
        let aspect_ratio = width as f32 / (height as f32);
        let half_vertical = f32::tan(fov_y.0 / 2.0);
        let half_horizontal = half_vertical * aspect_ratio;

        Camera {
            center: hcm::Point3::origin(),
            a: hcm::Vec3::new(half_horizontal / (width / 2) as f32, 0.0, 0.0),
            b: hcm::Vec3::new(0.0, -half_vertical / (height / 2) as f32, 0.0),
            c: hcm::Vec3::new(-half_horizontal, half_vertical, 1.0),
            width,
            height,
            orientation: [hcm::Vec3::xbase(), hcm::Vec3::ybase(), hcm::Vec3::zbase()],
        }
    }
    
    #[allow(dead_code)]
    pub fn look_at(&mut self, from: hcm::Point3, target: hcm::Point3, up: hcm::Vec3) {
        let forward = (target - from).hat();  // new z-axis
        let right = up.cross(forward).hat();          // new x-axis, equals to cross(y, z)
        let up = forward.cross(right);                // adjusted y-axis, equals to cross(z, x)
        
        self.orientation = [right, up, forward];
        self.center = from;
    }

    pub fn shoot_ray(&self, row: u32, col: u32, (dx, dy): (f32, f32)) -> Option<ray::Ray> {
        let x = col as f32 + dx.fract();
        let y = row as f32 + dy.fract();
        let c = Self::matvec(&self.orientation, self.c);
        let a = Self::matvec(&self.orientation, self.a);
        let b = Self::matvec(&self.orientation, self.b);
        if row >= self.height || col >= self.width {
            None
        } else {
            let dir = c + a * x + b * y;
            Some(ray::Ray::new(self.center, dir))
        }
    }
    
    fn matvec(m: &[hcm::Vec3; 3], v: hcm::Vec3) -> hcm::Vec3{
        m[0] * v[0] + m[1] * v[1] + m[2] * v[2]
    }
}
