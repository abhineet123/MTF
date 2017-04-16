FILE(REMOVE_RECURSE
  "CMakeFiles/syn"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/syn.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
