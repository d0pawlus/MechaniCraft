//! Indentation-based tree parser for SAM ASCII files.
//!
//! A keyed line `key:value` opens a node. Its children are the following keyed
//! lines with a larger indent (number of leading spaces). Lines without a colon
//! are stored as `rows` of the nearest preceding keyed line with a smaller indent.

use std::str::FromStr;

/// Errors produced while reading a SAM ASCII file.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum SamError {
    /// The first line is not `SAMASCII` (for example a binary file).
    #[error("not a SAM ASCII file: first line must be SAMASCII")]
    NotSamAscii,
    /// A required key is absent. `line` is the line of the node that was searched.
    #[error("missing key `{key}` in node at line {line}")]
    Missing { key: String, line: usize },
    /// A value cannot be parsed into the requested type.
    #[error("cannot parse `{key}` at line {line}")]
    BadNumber { key: String, line: usize },
    /// Content outside the supported subset of the format.
    #[error("unsupported {what} at line {line}")]
    Unsupported { what: String, line: usize },
}

/// One `key:value` line of a SAM file together with its nested content.
#[derive(Debug, Clone, PartialEq)]
pub struct SamNode {
    pub key: String,
    pub value: String,
    /// 1-based line number in the source file.
    pub line: usize,
    pub children: Vec<SamNode>,
    /// Lines without a colon that belong to this node (trimmed).
    pub rows: Vec<String>,
}

impl SamNode {
    /// First direct child with the given key.
    pub fn get(&self, key: &str) -> Option<&SamNode> {
        self.children.iter().find(|child| child.key == key)
    }

    /// All direct children with the given key, in file order.
    pub fn all<'a>(&'a self, key: &'a str) -> impl Iterator<Item = &'a SamNode> {
        self.children.iter().filter(move |child| child.key == key)
    }

    /// Parses the value of the first direct child with the given key.
    pub fn num<T: FromStr>(&self, key: &str) -> Result<T, SamError> {
        let child = self.get(key).ok_or_else(|| SamError::Missing {
            key: key.to_owned(),
            line: self.line,
        })?;
        child.value.parse::<T>().map_err(|_| SamError::BadNumber {
            key: key.to_owned(),
            line: child.line,
        })
    }
}

/// A parsed SAM file: format version from line 2 and the top-level nodes.
#[derive(Debug, Clone, PartialEq)]
pub struct SamTree {
    pub format_version: u32,
    pub root: Vec<SamNode>,
}

impl SamTree {
    /// First top-level node with the given key.
    pub fn get(&self, key: &str) -> Option<&SamNode> {
        self.root.iter().find(|node| node.key == key)
    }

    /// All top-level nodes with the given key, in file order.
    pub fn all<'a>(&'a self, key: &'a str) -> impl Iterator<Item = &'a SamNode> {
        self.root.iter().filter(move |node| node.key == key)
    }
}

/// Parses the bytes of a SAM ASCII file into a tree.
///
/// Invalid UTF-8 is replaced lossily. Parsing stops at `ENDSAM`.
pub fn parse_tree(bytes: &[u8]) -> Result<SamTree, SamError> {
    let text = String::from_utf8_lossy(bytes);
    let mut lines = text.lines().enumerate().map(|(index, line)| (index + 1, line));

    match lines.next() {
        Some((_, line)) if line.trim() == "SAMASCII" => {}
        _ => return Err(SamError::NotSamAscii),
    }

    let format_version = match lines.next() {
        Some((line_no, line)) => line.trim().parse::<u32>().map_err(|_| SamError::BadNumber {
            key: "format_version".to_owned(),
            line: line_no,
        })?,
        None => {
            return Err(SamError::Missing {
                key: "format_version".to_owned(),
                line: 2,
            });
        }
    };

    let mut root = Vec::new();
    // Open nodes as (indent, node), outermost first. Indents strictly increase.
    let mut stack: Vec<(usize, SamNode)> = Vec::new();

    for (line_no, raw) in lines {
        let content = raw.trim();
        if content == "ENDSAM" {
            break;
        }
        if content.is_empty() || content == "ENDOBJ" {
            continue;
        }

        let indent = raw.len() - raw.trim_start_matches(' ').len();

        match content.split_once(':') {
            Some((key, value)) => {
                // Close every open node that is not shallower than this line.
                while stack.last().is_some_and(|(open, _)| *open >= indent) {
                    close_top(&mut stack, &mut root);
                }
                stack.push((
                    indent,
                    SamNode {
                        key: key.trim().to_owned(),
                        value: value.trim().to_owned(),
                        line: line_no,
                        children: Vec::new(),
                        rows: Vec::new(),
                    },
                ));
            }
            None => {
                // A row belongs to the nearest open node with a smaller indent.
                // Rows without such a node (e.g. header tokens) are ignored.
                if let Some((_, parent)) = stack.iter_mut().rev().find(|(open, _)| *open < indent) {
                    parent.rows.push(content.to_owned());
                }
            }
        }
    }

    while !stack.is_empty() {
        close_top(&mut stack, &mut root);
    }

    Ok(SamTree { format_version, root })
}

/// Pops the innermost open node and attaches it to its parent, or to the root list.
fn close_top(stack: &mut Vec<(usize, SamNode)>, root: &mut Vec<SamNode>) {
    if let Some((_, closed)) = stack.pop() {
        match stack.last_mut() {
            Some((_, parent)) => parent.children.push(closed),
            None => root.push(closed),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const EXCERPT: &str = "SAMASCII
802
BUILDNRREAD:8.0.1.161
CREATORTYPE_PRO
 ELEMGROUPS:
  ELEMGRP:
   NAME:1
   ELEMLIST:
    BEAM:
     NR:1
     OUTELEMFLAGS:
      OUTFLAGS:0
      ATLEFTFL:4294967295
      ENDOBJ
     OPTIMIZATIONS:3
      0  0.00000000000000E+0000  0.00000000000000E+0000
      0  0.00000000000000E+0000  0.00000000000000E+0000
      0  0.00000000000000E+0000  0.00000000000000E+0000
     N1:1
     N2:2
     ENDOBJ
DOFLIST:
 FIRSTANGLENR:1000
 NODE:
  NR:1
  FIXAXIS:3
  X:-0.0568627450977601
  Y:0.480392156862308
 ANGLE:
  NR:1001
  FIXAXIS:0
  A:0
INPUT:
 MAXIM:10
 MOTIONBLOCK:
  EDNR:3
  KIND:1
   CURVELIST:
   CLINEAR:
    NSTEPS:36
    CTIME:1
    MOTION:6.28318512
    ENDOBJ
WCS:
 DEFAULT:
  -9.93233686916710E-0002  9.61406402354605E-0001  8.34810350368571E-0001  3.03968763346986E-0002
GRAPHOPTIONS:
 TITLE:Graph of Selected items
 LB_LEFT:
ENDSAM
";

    fn parse(text: &str) -> Result<SamTree, SamError> {
        parse_tree(text.as_bytes())
    }

    /// Depth-first search for the first node with the given key.
    fn find<'a>(nodes: &'a [SamNode], key: &str) -> Option<&'a SamNode> {
        for node in nodes {
            if node.key == key {
                return Some(node);
            }
            if let Some(found) = find(&node.children, key) {
                return Some(found);
            }
        }
        None
    }

    #[test]
    fn parses_nesting_by_indent() {
        let tree = parse(
            "SAMASCII\n802\nDOFLIST:\n NODE:\n  NR:1\n  X:-0.05\n ANGLE:\n  NR:1001\nENDSAM\n",
        )
        .unwrap();
        let node = tree.root[0].all("NODE").next().unwrap();
        assert_eq!(tree.root[0].all("NODE").count(), 1);
        assert_eq!(node.num::<f64>("X").unwrap(), -0.05);
        assert_eq!(tree.root[0].get("ANGLE").unwrap().num::<u32>("NR").unwrap(), 1001);
    }

    #[test]
    fn rows_attach_to_parent() {
        let tree = parse(EXCERPT).unwrap();
        let optimizations = find(&tree.root, "OPTIMIZATIONS").unwrap();
        assert_eq!(optimizations.rows.len(), 3);
        assert_eq!(optimizations.rows[0], "0  0.00000000000000E+0000  0.00000000000000E+0000");
    }

    #[test]
    fn rejects_binary() {
        let err = parse_tree(&[0x00, 0x01]).unwrap_err();
        assert_eq!(err, SamError::NotSamAscii);
    }

    #[test]
    fn parses_non_utf8_bytes_lossily() {
        let mut bytes = b"SAMASCII\n802\nPROJECTNOTES:".to_vec();
        bytes.extend_from_slice(&[0xB3]);
        bytes.extend_from_slice(b"ODA\nENDSAM\n");
        let tree = parse_tree(&bytes).unwrap();
        let notes = tree.get("PROJECTNOTES").unwrap();
        assert!(notes.value.contains('\u{FFFD}'));
        assert!(notes.value.ends_with("ODA"));
    }

    #[test]
    fn missing_key_reports_line() {
        let tree = parse("SAMASCII\n802\nNODE:\n X:1.0\nENDSAM\n").unwrap();
        let node = tree.get("NODE").unwrap();
        assert_eq!(node.line, 3);
        assert_eq!(
            node.num::<u32>("NR").unwrap_err(),
            SamError::Missing { key: "NR".to_owned(), line: 3 }
        );
    }

    #[test]
    fn parses_excerpt_structure() {
        let tree = parse(EXCERPT).unwrap();
        assert_eq!(tree.format_version, 802);
        assert_eq!(tree.get("BUILDNRREAD").unwrap().value, "8.0.1.161");
        let node = find(&tree.root, "NODE").unwrap();
        assert_eq!(node.num::<f64>("X").unwrap(), -0.0568627450977601);
        assert_eq!(find(&tree.root, "ATLEFTFL").unwrap().value, "4294967295");
        assert_eq!(find(&tree.root, "ATLEFTFL").unwrap().value.parse::<u64>().unwrap(), 4294967295);
        let clinear = find(&tree.root, "CLINEAR").unwrap();
        assert_eq!(clinear.num::<u32>("NSTEPS").unwrap(), 36);
        assert_eq!(clinear.num::<f64>("MOTION").unwrap(), 6.28318512);
        assert_eq!(find(&tree.root, "DEFAULT").unwrap().rows.len(), 1);
        let graph = tree.get("GRAPHOPTIONS").unwrap();
        assert_eq!(graph.get("TITLE").unwrap().value, "Graph of Selected items");
        assert_eq!(graph.get("LB_LEFT").unwrap().value, "");
        assert_eq!(tree.all("DOFLIST").count(), 1);
    }

    #[test]
    fn accepts_crlf_and_splits_value_on_first_colon() {
        let text = "SAMASCII\n802\nTITLE:a:b c \nENDSAM\n".replace('\n', "\r\n");
        let tree = parse(&text).unwrap();
        assert_eq!(tree.get("TITLE").unwrap().value, "a:b c");
    }

    #[test]
    fn rejects_bad_format_version() {
        assert_eq!(
            parse("SAMASCII\nabc\nENDSAM\n").unwrap_err(),
            SamError::BadNumber { key: "format_version".to_owned(), line: 2 }
        );
    }
}
