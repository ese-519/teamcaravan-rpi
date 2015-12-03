FILE(REMOVE_RECURSE
  "CMakeFiles/tf_generate_messages_py"
  "/home/ubuntu/hemo_code/new_code/wksp/devel/lib/python2.7/dist-packages/tf/msg/_tfMessage.py"
  "/home/ubuntu/hemo_code/new_code/wksp/devel/lib/python2.7/dist-packages/tf/srv/_FrameGraph.py"
  "/home/ubuntu/hemo_code/new_code/wksp/devel/lib/python2.7/dist-packages/tf/msg/__init__.py"
  "/home/ubuntu/hemo_code/new_code/wksp/devel/lib/python2.7/dist-packages/tf/srv/__init__.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/tf_generate_messages_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
