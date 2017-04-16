FILE(REMOVE_RECURSE
  "CMakeFiles/all"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/all.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
