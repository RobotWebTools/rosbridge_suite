# $URL$
# $Rev$

# PyPNG documentation build configuration file, created by
# sphinx-quickstart on Mon Mar 16 13:12:26 2009.
# Then brutally hacked down by drj.
# See http://sphinx.pocoo.org/config.html

import sys, os

# So that local modules get picked up, and picked up first.
sys.path.insert(0, os.path.abspath('../code'))
# So that setup.py can be picked up and used for its conf member
sys.path.insert(0, os.path.abspath('..'))

# Expecting to find ../setup.py
from setup import conf

# General configuration
# ---------------------

extensions = ['sphinx.ext.autodoc']
templates_path = []
source_suffix = '.rst'
master_doc = 'index'
project = u'PyPNG'
copyright = u'2009, ' + conf['author']
release = conf['version']
version = release[:release.rfind('.')]
language='en'
today_fmt = '%Y-%m-%d'
exclude_trees = ['build']


# Options for HTML output
# -----------------------

html_static_path = []
html_last_updated_fmt = '%Y-%m-%dT%H:%M:%S'
htmlhelp_basename = 'PyPNGdoc'


# Options for LaTeX output
# ------------------------

# The paper size ('letter' or 'a4').
latex_paper_size = 'a4'

# The font size ('10pt', '11pt' or '12pt').
#latex_font_size = '10pt'

# Grouping the document tree into LaTeX files. List of tuples
# (source start file, target name, title, author, document class [howto/manual]).
latex_documents = [
  ('index', 'PyPNG.tex', ur'PyPNG Documentation',
   ur'David Jones', 'manual'),
]

# The name of an image file (relative to this directory) to place at the top of
# the title page.
#latex_logo = None

# http://sphinx.pocoo.org/ext/autodoc.html?highlight=__init__
autoclass_content='both'
