# Configuration file for the Sphinx documentation builder.

# -- Project information

project = 'ARIAC'
copyright = 'NIST'
author = 'Pavel'

release = '1.0'
version = '0.1.0'


# -- General configuration

extensions = [
    'hoverxref.extension',
    'myst_parser',
    'sphinx.ext.mathjax',
    'sphinx_rtd_theme',
    'sphinx.ext.autosectionlabel',
    'sphinx.ext.todo',
    # External stuff
    "myst_parser",
    "sphinx_copybutton",
    'sphinx.ext.intersphinx',
    'sphinxemoji.sphinxemoji'
]


# use language set by highlight directive if no language is set by role
inline_highlight_respect_highlight = False

# use language set by highlight directive if no role is set
inline_highlight_literals = False

todo_include_todos = True

templates_path = ['_templates']

# Make sure the target is unique
autosectionlabel_prefix_document = True

# intersphinx_mapping = {
#     'python': ('https://docs.python.org/3/', None),
#     'sphinx': ('https://www.sphinx-doc.org/en/master/', None),
# }

intersphinx_mapping = {
    'readthedocs': ('https://docs.readthedocs.io/en/stable/', None),
    'sphinx': ('https://www.sphinx-doc.org/en/master/', None),
    'sympy': ('https://docs.sympy.org/latest/', None),
    'numpy': ('https://numpy.org/doc/stable/', None),
    'python': ('https://docs.python.org/3/', None),
}

hoverxref_roles = [
    'numref',
    'confval',
    'setting',
    "doc",
    'term',
]
hoverxref_intersphinx = [
    'readthedocs',
    'sphinx',
    'sympy',
    'numpy',
    'python',
]
hoverxref_intersphinx_types = {
    'readthedocs': 'modal',
    'sphinx': 'tooltip',
}

intersphinx_disabled_domains = ['std']

# hoverxref_roles = [
#     'confval',
#     'term',
# ]
hoverxref_role_types = {
    'hoverxref': 'tooltip',
    'ref': 'modal',
    'doc': 'modal',
    'confval': 'tooltip',
    'mod': 'modal',
    'class': 'modal',
    'obj': 'tooltip',
}
# hoverxref_domains = [
#     'py',
#     'cite',
# ]

hoverxref_tooltip_maxwidth = 650
hoverxref_auto_ref = True


sphinxemoji_style = 'twemoji'
html_theme = 'sphinx_rtd_theme'

# The name of the Pygments (syntax highlighting) style to use.
pygments_style = 'tango'

rst_prolog = """
 .. include:: <s5defs.txt>
 
 .. role:: topic
    :class: topic

 .. role:: rosservice
    :class: rosservice
    
 .. role:: yamlname(code)
    :class: yamlname
    
 .. role:: yaml(code)
    :language: yaml
    :class: highlight
    
 .. role:: tuto
    :class: tuto
    
 .. role:: console(code)
    :language: console
    :class: highlight
    
 .. role:: bash
    :language: bash
    :class: highlight

 .. role:: cpp(code)
    :language: cpp
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
