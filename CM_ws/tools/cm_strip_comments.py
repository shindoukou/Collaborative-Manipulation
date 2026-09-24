#!/usr/bin/env python3
import io
import re
import tokenize
from pathlib import Path


ROOT = Path("/home/mas-2/3rd_party/CM_ws/src")
PACKAGES = (ROOT / "cm_dnmpc", ROOT / "cm_kirara_ball", ROOT / "cm_kirara_box")


def strip_python(text):
    source = io.StringIO(text)
    output = []
    try:
        tokens = tokenize.generate_tokens(source.readline)
        for token in tokens:
            if token.type == tokenize.COMMENT and not (token.start[0] == 1 and token.string.startswith("#!")):
                token = tokenize.TokenInfo(token.type, "", token.start, token.end, token.line)
            output.append(token)
        return tokenize.untokenize(output)
    except (tokenize.TokenError, IndentationError):
        return text


def strip_cpp(text):
    output = []
    index = 0
    size = len(text)
    while index < size:
        if text.startswith("//", index):
            output.append(" ")
            index += 2
            while index < size and text[index] not in "\r\n":
                index += 1
            continue
        if text.startswith("/*", index):
            output.append(" ")
            index += 2
            while index < size and not text.startswith("*/", index):
                if text[index] in "\r\n":
                    output.append(text[index])
                index += 1
            index = min(size, index + 2)
            continue
        if text.startswith('R"', index):
            delimiter_end = text.find("(", index + 2, min(size, index + 20))
            if delimiter_end != -1:
                delimiter = text[index + 2:delimiter_end]
                terminator = ")" + delimiter + '"'
                literal_end = text.find(terminator, delimiter_end + 1)
                if literal_end != -1:
                    literal_end += len(terminator)
                    output.append(text[index:literal_end])
                    index = literal_end
                    continue
        if text[index] in ("\"", "'"):
            quote = text[index]
            start = index
            index += 1
            while index < size:
                if text[index] == "\\":
                    index += 2
                    continue
                if text[index] == quote:
                    index += 1
                    break
                index += 1
            output.append(text[start:index])
            continue
        output.append(text[index])
        index += 1
    return "".join(output)


def strip_hash(text):
    lines = []
    for number, line in enumerate(text.splitlines(keepends=True), 1):
        if number == 1 and line.startswith("#!"):
            lines.append(line)
            continue
        quote = None
        escaped = False
        cut = None
        for index, char in enumerate(line):
            if escaped:
                escaped = False
                continue
            if char == "\\" and quote is not None:
                escaped = True
                continue
            if char in ("\"", "'"):
                if quote is None:
                    quote = char
                elif quote == char:
                    quote = None
                continue
            if char == "#" and quote is None:
                cut = index
                break
        if cut is None:
            lines.append(line)
        else:
            ending = "\n" if line.endswith("\n") else ""
            lines.append(line[:cut].rstrip() + ending)
    return "".join(lines)


def strip_xml(text):
    return re.sub(r"<!--.*?-->", lambda match: "\n" * match.group(0).count("\n"), text, flags=re.DOTALL)


for package in PACKAGES:
    for path in package.rglob("*"):
        if not path.is_file() or "generated" in path.parts or "backup_" in str(path) or "__pycache__" in path.parts:
            continue
        if path.suffix == ".py":
            method = strip_python
        elif path.suffix in (".cpp", ".cc", ".cxx", ".h", ".hpp"):
            method = strip_cpp
        elif path.suffix in (".xml", ".launch", ".urdf", ".xacro"):
            method = strip_xml
        elif path.suffix in (".yaml", ".yml") or path.name == "CMakeLists.txt":
            method = strip_hash
        else:
            continue
        data = path.read_text(encoding="utf-8", errors="surrogateescape")
        updated = method(data)
        if updated != data:
            path.write_text(updated, encoding="utf-8", errors="surrogateescape")
