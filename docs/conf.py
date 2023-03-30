# Configuration file for the Sphinx documentation builder.

# -- Project information

project = 'ARIAC'
copyright = 'NIST'
author = 'Pavel'

release = '1.0'
version = '0.1.0'


# -- General configuration

extensions = [
    'myst_parser',
    'sphinx.ext.mathjax',
    'sphinx_rtd_theme',
    'sphinx.ext.autosectionlabel',
    'sphinx.ext.todo',
    # External stuff
    "myst_parser",
    "sphinx_copybutton",
    'sphinxcontrib.inlinesyntaxhighlight'
]

todo_include_todos = True

templates_path = ['_templates']

# Make sure the target is unique
autosectionlabel_prefix_document = True

intersphinx_mapping = {
    'python': ('https://docs.python.org/3/', None),
    'sphinx': ('https://www.sphinx-doc.org/en/master/', None),
}
intersphinx_disabled_domains = ['std']

html_theme = 'sphinx_rtd_theme'

# The name of the Pygments (syntax highlighting) style to use.
pygments_style = 'tango'

rst_prolog = """
 .. include:: <s5defs.txt>
 
 .. role:: yaml(code)
    :language: yaml
    :class: highlight

 .. role:: python(code)
    :language: python
    :class: highlight
    
 .. role:: bash(code)
    :language: bash
    :class: highlight    
 """

source_suffix = ['.rst', '.md']
html_static_path = ['custom']
html_css_files = [
    'css/custom.css',
    'css/hack.css',
]
html_js_files = [
    'js/custom.js'
]


# -- Options for copy button -------------------------------------------------------
#
copybutton_prompt_text = r">>> |\.\.\. |\$ |In \[\d*\]: | {2,5}\.\.\.: | {5,8}: "
copybutton_prompt_is_regexp = True
copybutton_line_continuation_character = "\\"
copybutton_here_doc_delimiter = "EOT"
copybutton_selector = "div:not(.no-copybutton) > div.highlight > pre"
numfig = True


# -- Options for EPUB output
epub_show_urls = 'footnote'
