use core::panic;

use math::hcm::{Point3, Vec3};
use crate::ast::*;
use crate::token::Token;

macro_rules! raise_syntax_error {
    ($($arg:tt)*) => {{
        let res = std::fmt::format(format_args!($($arg)*));
        panic!("pbrs syntax error [{}:{}]: {}", file!(), line!(), res)
    }}
}

pub struct Parser {
    iter: std::vec::IntoIter<Token>,
    peek: Token,
}

impl Parser {
    // fn dont_call_me(self: &mut Self) {
    //     let dummy : Token = self.peek.into();
    //     self.peek = self.iter.next().unwrap();
    // }
    #[allow(dead_code)]
    pub fn parse_scene(&mut self) -> Scene {
        let mut options = vec![];
        while self.peek.starts_scene_option() {
            options.push(self.parse_scene_option());
        }
        // println!("Scene options = {}", options);
        self.consume_next(Token::KwWorldBegin);
        let world_items = self.parse_world_item_list();
        self.consume_next(Token::KwWorldEnd);

        Scene {
            options,
            items: world_items,
        }
    }

    fn parse_world_item(&mut self) -> WorldItem {
        match self.peek {
            Token::KwIdentity
            | Token::KwTranslate
            | Token::KwScale
            | Token::KwRotate
            | Token::KwLookAt
            | Token::KwCoordSysTransform
            | Token::KwCoordinateSystem
            | Token::KwTransform
            | Token::KwConcatTransform => {
                let t = self.parse_transform();
                WorldItem::Transform(t)
            }
            Token::KwShape => {
                self.goto_next();
                if let Token::QuotedString(implementation) = self.peek.clone() {
                    self.goto_next();
                    let parameter_set = self.parse_parameter_list();
                    WorldItem::Shape(implementation, parameter_set)
                } else {
                    raise_syntax_error!("unexpected token after Shape keyword {:?}", self.peek)
                }
            }
            Token::KwMaterial => {
                self.goto_next();
                if let Token::QuotedString(implementation) = self.peek.clone() {
                    self.goto_next();
                    WorldItem::Material(implementation, self.parse_parameter_list())
                } else {
                    raise_syntax_error!("unexpected token after Material keyword: {:?}", self.peek)
                }
            }
            Token::KwLightSource => {
                self.goto_next();
                if let Token::QuotedString(implementation) = self.peek.clone() {
                    self.goto_next();
                    WorldItem::Light(implementation, self.parse_parameter_list())
                } else {
                    raise_syntax_error!("unexpected token after LightSrc keyword: {:?}", self.peek)
                }
            }
            Token::KwAreaLightSource => {
                self.goto_next();
                if let Token::QuotedString(implementation) = self.peek.clone() {
                    self.goto_next();
                    WorldItem::AreaLight(implementation, self.parse_parameter_list())
                } else {
                    raise_syntax_error!("unexpected token after KwAreaLight: {:?}", self.peek)
                }
            }
            Token::KwTexture => {
                self.goto_next();
                let texture_name = self.get_next_quoted_string();
                let texture_type = self.get_next_quoted_string();
                let implementation = self.get_next_quoted_string();
                WorldItem::Texture(
                    implementation,
                    texture_type,
                    texture_name,
                    self.parse_parameter_list(),
                )
            }
            Token::KwMakeNamedMaterial => {
                self.goto_next();
                let mtl_name = self.get_next_quoted_string();
                WorldItem::MakeMaterial(mtl_name, self.parse_parameter_list())
            }
            Token::KwObjectInstance => WorldItem::ObjectInstance(self.get_next_quoted_string()),
            Token::KwAttributeBegin => {
                self.goto_next();
                if self.peek == Token::KwAttributeEnd {
                    WorldItem::AttributeBlock(vec![])
                } else if self.peek == Token::KwObjectBegin {
                    self.goto_next();
                    let object_name = self.get_next_quoted_string();
                    let world_item_list = self.parse_world_item_list();
                    if self.peek == Token::KwAttributeEnd {
                        self.goto_next();
                        self.consume_next(Token::KwObjectEnd);
                        WorldItem::ObjectBlock(object_name, world_item_list)
                    } else if self.peek == Token::KwObjectEnd {
                        self.goto_next();
                        self.consume_next(Token::KwAttributeEnd);
                        WorldItem::ObjectBlock(object_name, world_item_list)
                    } else {
                        raise_syntax_error!(
                            "unexpected token before ending building an object: {:?}",
                            self.peek
                        )
                    }
                } else {
                    let items = self.parse_world_item_list();
                    self.consume_next(Token::KwAttributeEnd);
                    WorldItem::AttributeBlock(items)
                }
            }
            Token::KwObjectBegin => {
                self.goto_next();
                let object_name = self.get_next_quoted_string();
                let world_item_list = self.parse_world_item_list();
                self.consume_next(Token::KwObjectEnd);
                WorldItem::ObjectBlock(object_name, world_item_list)
            }
            Token::KwTransformBegin => {
                self.goto_next();
                let item_list = self.parse_world_item_list();
                self.consume_next(Token::KwTransformEnd);
                WorldItem::TransformBlock(item_list)
            }
            Token::KwNamedMaterial => {
                self.goto_next();
                WorldItem::MaterialInstance(self.get_next_quoted_string())
            }
            _ => self.raise_syntax_error(
                format!("invalid token for starting an item: {:?}", self.peek).as_str(),
            ),
        }
    }

    /// Consumes the upcoming tokens as many as possible as long as they start with a world item
    /// keyword, and returns the list of world items as a `Vec<WorldItem>`.
    ///
    /// The returned `Vec` might be empty in which case no tokens are consumed.
    fn parse_world_item_list(&mut self) -> Vec<WorldItem> {
        let mut items = vec![];
        while self.peek.starts_world_item() {
            items.push(self.parse_world_item());
        }
        items
    }

    fn parse_scene_option(&mut self) -> SceneWideOption {
        match self.peek {
            Token::KwCamera => {
                self.goto_next();
                let implementation = self.get_next_quoted_string();
                let parameter_list = self.parse_parameter_list();
                SceneWideOption::Camera(implementation, parameter_list)
            }
            Token::KwSampler => {
                self.goto_next();
                let implementation = self.get_next_quoted_string();
                let parameter_list = self.parse_parameter_list();
                SceneWideOption::Sampler(implementation, parameter_list)
            }
            Token::KwFilm => {
                self.goto_next();
                let implementation = self.get_next_quoted_string();
                let parameter_list = self.parse_parameter_list();
                SceneWideOption::Film(implementation, parameter_list)
            }
            Token::KwFilter => {
                self.goto_next();
                let implementation = self.get_next_quoted_string();
                let parameter_list = self.parse_parameter_list();
                SceneWideOption::Filter(implementation, parameter_list)
            }
            Token::KwIntegrator => {
                self.goto_next();
                let implementation = self.get_next_quoted_string();
                let parameter_list = self.parse_parameter_list();
                SceneWideOption::Integrator(implementation, parameter_list)
            }
            Token::KwAccelerator => {
                self.goto_next();
                let implementation = self.get_next_quoted_string();
                let parameter_list = self.parse_parameter_list();
                SceneWideOption::Accel(implementation, parameter_list)
            }
            Token::KwLookAt => SceneWideOption::Transform(self.parse_transform()),
            ref t if t.starts_transform() => SceneWideOption::Transform(self.parse_transform()),
            _ => raise_syntax_error!("incorrect token to start a scene option: {:?}", self.peek),
        }
    }

    /// Parses the next few tokens as a parameter, assuming the next peek is a string literal.
    /// Raises a syntax error otherwise.
    fn parse_parameter(&mut self) -> (String, ArgValue) {
        let key = if let Token::QuotedString(key) = self.peek.clone() {
            key
        } else {
            self.raise_syntax_error("parameter list doesn't start with a string literal");
        };

        self.goto_next();
        let value = match self.peek.take() {
            Token::LBracket => {
                self.goto_next();
                let param = match self.peek {
                    Token::Float(_) => {
                        let number_list = self.parse_number_list();
                        assert!(!number_list.is_empty());
                        if number_list.len() == 1 {
                            ArgValue::Number(number_list[0])
                        } else {
                            ArgValue::Numbers(number_list)
                        }
                    }
                    Token::QuotedString(_) => ArgValue::String(self.get_next_quoted_string()),
                    _ => self.raise_syntax_error("only numbers or quoted strings allowed"),
                };
                self.consume_next(Token::RBracket);
                param
            }
            Token::QuotedString(s) => {
                self.goto_next();
                ArgValue::String(s)
            }
            Token::Float(f) => {
                self.goto_next();
                ArgValue::Number(f)
            }
            _ => self.raise_syntax_error("unexpected token after key"),
        };
        (key, value)
    }

    /// Collect parameters in the input stream as many as possible and returns the collection as a Vec.
    fn parse_parameter_list(&mut self) -> ParameterSet {
        let mut parameters = std::collections::HashMap::<String, ArgValue>::new();
        while let Token::QuotedString(_) = self.peek {
            let (k, v) = self.parse_parameter();
            parameters.insert(k, v);
        }
        ParameterSet(parameters)
    }

    fn parse_transform(&mut self) -> Transform {
        let keyword = self.peek.clone();
        self.goto_next();
        match keyword {
            Token::KwIdentity => Transform::Identity,
            Token::KwTranslate => {
                let numbers = self.parse_number_list();
                if let [x, y, z] = numbers.as_slice() {
                    let translation = Vec3::new(*x, *y, *z);
                    Transform::Translate(translation)
                } else {
                    raise_syntax_error!("wrong number of numbers after translation");
                }
            }
            Token::KwScale => {
                let raw_numbers = self.get_next_3_numbers();
                let [x, y, z] = raw_numbers;
                Transform::Scale(Vec3::new(x, y, z))
            }
            Token::KwRotate => {
                if let Token::Float(deg) = self.peek {
                    self.goto_next();
                    let raw_numbers = self.get_next_3_numbers();
                    let [x, y, z] = raw_numbers;
                    Transform::Rotate(Vec3::new(x, y, z), math::new_deg(deg))
                } else {
                    self.raise_syntax_error("4th number expected in rotate");
                }
            }
            Token::KwLookAt => {
                let raw = self.parse_number_list();
                if raw.len() != 9 {
                    self.raise_syntax_error("wrong numbers of floats in LookAt");
                } else {
                    Transform::LookAt(
                        Point3::new(raw[0], raw[1], raw[2]),
                        Point3::new(raw[3], raw[4], raw[5]),
                        Vec3::new(raw[6], raw[7], raw[8]),
                    )
                }
            }
            Token::KwCoordSysTransform => {
                Transform::CoordSys(self.get_next_quoted_string())
            }
            Token::KwTransform
            | Token::KwConcatTransform
            | Token::KwCoordinateSystem => unimplemented!("sorry"),
            _ => panic!(
                "unexpected token {:?}, {:?}",
                keyword,
                self.iter.clone().enumerate()
            ),
        }
    }

    fn parse_number_list(&mut self) -> Vec<f32> {
        let mut numbers = vec![];
        while let Token::Float(number) = self.peek {
            numbers.push(number);
            self.goto_next();
        }
        assert!(!matches!(self.peek, Token::Float(_)));
        numbers
    }
    fn get_next_3_numbers(&mut self) -> [f32; 3] {
        let mut numbers = [0.0f32; 3];
        for i in 0..3 {
            if let Token::Float(x) = self.peek {
                numbers[i] = x;
                self.goto_next();
            } else {
                self.raise_syntax_error("float needed");
            }
        }
        numbers
    }
    fn get_next_quoted_string(&mut self) -> String {
        if let Token::QuotedString(s) = self.peek.take() {
            self.goto_next();
            s
        } else {
            raise_syntax_error!("expected quoted string")
        }
    }

    #[allow(dead_code)]
    fn is_at_end(&self) -> bool {
        if self.iter.clone().peekable().peek().is_none() {
            assert_eq!(self.peek, Token::End);
        }
        self.peek == Token::End
    }

    fn goto_next(&mut self) {
        if self.peek != Token::End {
            self.peek = self.iter.next().unwrap()
        } else {
            self.peek = Token::End
        }
    }

    fn consume_next(&mut self, t: Token) {
        if self.peek == t {
            self.goto_next()
        } else {
            panic!("expected next token = {:?} (actually {:?})", t, self.peek)
        }
    }

    fn raise_syntax_error(&self, msg: &str) -> ! {
        panic!("syntax error: {}", msg)
    }

    pub fn new(mut iter: std::vec::IntoIter<Token>) -> Self {
        let peek = iter.next().unwrap();

        Self { iter, peek }
    }
}
