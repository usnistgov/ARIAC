from typing import Any

from nicegui import ui 

class LatexTable:
    def __init__(
        self,
        title: str,
        columns: list[dict[str, Any]],
        rows: list[dict[str, Any]],
        latex_column_names: list[str] = [],
    ):
        """Creates a NiceGUI table that supports LaTeX rendering using MathJax.

        Args:
            title (str): Table title
            columns (list): Column definitions (NiceGUI format)
            rows (list): Data rows
            latex_column_names (list): List of column `name` fields that contain LaTeX strings
        """
        # Mark specified columns as HTML to allow MathJax rendering
        for col in columns:
            if col['name'] in latex_column_names:
                col['html'] = True  # allows HTML/MathJax inside table cell

        # Inject MathJax once per app
        ui.add_head_html('''
        <script>
        if (!window.MathJax) {
            window.MathJax = {
              tex: {inlineMath: [['$', '$'], ['\\\\(', '\\\\)']]},
              svg: {fontCache: 'global'}
            };
            let script = document.createElement('script');
            script.src = 'https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-svg.js';
            script.async = true;
            document.head.appendChild(script);
        }
        </script>
        ''')

        # Build table
        self.table = ui.table(
            title=title,
            columns=columns,
            rows=rows,
        )

        # Trigger MathJax rendering after table loads
        ui.timer(0.5, lambda: ui.run_javascript('MathJax.typeset();'), once=True)