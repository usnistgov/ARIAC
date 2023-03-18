# Configuration file for the Sphinx documentation builder.

# -- Project information

project = 'ARIAC'
copyright = 'NIST'
author = 'Pavel'

release = '0.1'
version = '0.1.0'

# -- General configuration

extensions = [
    'myst_parser',
    'sphinx.ext.mathjax',
    'sphinx_rtd_theme',
    'sphinx.ext.autosectionlabel'
]

# Make sure the target is unique
autosectionlabel_prefix_document = True

intersphinx_mapping = {
    'python': ('https://docs.python.org/3/', None),
    'sphinx': ('https://www.sphinx-doc.org/en/master/', None),
}
intersphinx_disabled_domains = ['std']

templates_path = ['_templates']



# -- Options for HTML output

html_theme = 'sphinx_rtd_theme'
# The name of the Pygments (syntax highlighting) style to use.

pygments_style = 'colorful'

source_suffix = ['.rst', '.md']

html_static_path = ['custom']

html_css_files = [
    'css/custom.css',
]

numfig = True

# -- Options for EPUB output
epub_show_urls = 'footnote'