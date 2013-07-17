FILE(REMOVE_RECURSE
  "msg_gen"
  "msg_gen"
  "src/tutorialROSOpenCV/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "src/tutorialROSOpenCV/msg/__init__.py"
  "src/tutorialROSOpenCV/msg/_Stringts.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
