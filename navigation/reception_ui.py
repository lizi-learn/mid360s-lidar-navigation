"""One UI renderer for direct HTTP and the existing G12 embedded document."""
from pathlib import Path

def reception_html(root):
    root=Path(root)
    return (root/'reception.html').read_text().replace('/*RECEPTION_TOOLS*/',
        (root/'reception_hash.js').read_text()+'\n'+(root/'reception_tools.js').read_text())
