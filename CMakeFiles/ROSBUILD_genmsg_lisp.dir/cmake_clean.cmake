FILE(REMOVE_RECURSE
  "msg_gen"
  "msg_gen"
  "src/tutorialROSOpenCV/msg"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "msg_gen/lisp/Stringts.lisp"
  "msg_gen/lisp/_package.lisp"
  "msg_gen/lisp/_package_Stringts.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
