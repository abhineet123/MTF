FILE(REMOVE_RECURSE
  "CMakeFiles/install_all"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/install_all.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
