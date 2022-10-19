from xml.dom import minidom
from svg.path import parse_path
from svg.path.path import Line


def main():
    with open("C:\\Users\\eliotkaplan\\Desktop\\drawing.svg") as f:
        doc = minidom.parse(f)
    path_strings = [path.getAttribute('d') for path
                    in doc.getElementsByTagName('path')]
    doc.unlink()
    # print the line draw commands
    for path_string in path_strings:
        path = parse_path(path_string)
        for e in path:
            print(e)


if __name__ == '__main__':
    main()
