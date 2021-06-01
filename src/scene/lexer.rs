use logos::Logos;
use std::{ffi::OsString, io::Read};

use crate::scene::token::Token;

pub struct Lexer {
    root_file: OsString,
    root_dir: std::path::PathBuf,
}

impl Lexer {
    pub fn from_file(path: &str) -> Result<Lexer, &str> {
        let path = std::path::Path::new(path);

        match path.parent() {
            None => Err("no parent directory available"),
            Some(parent) => {
                let root_file = path.file_name().unwrap().to_owned();
                Ok(Lexer {
                    root_file,
                    root_dir: std::path::PathBuf::from(parent),
                })
            }
        }
    }

    pub fn read_tokens(&self) -> Vec<Token> {
        let mut dir = self.root_dir.clone();
        dir.push(self.root_file.clone());
        let mut in_file = std::fs::File::open(dir).expect("open file failed");
        let mut content = String::new();
        in_file
            .read_to_string(&mut content)
            .expect("reading file failed");

        let tokenizer = Token::lexer(&content);

        tokenizer.collect::<Vec<_>>()
    }
}
