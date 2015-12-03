execute_process(COMMAND "/home/ubuntu/hemo_code/new_code/wksp/build/geometry/tf_conversions/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/ubuntu/hemo_code/new_code/wksp/build/geometry/tf_conversions/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
