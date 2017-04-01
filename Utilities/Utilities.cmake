set(MTF_UTILITIES histUtils warpUtils imgUtils miscUtils graphUtils spiUtils)
addPrefixAndSuffix("${MTF_UTILITIES}" "Utilities/src/" ".cc" MTF_UTILITIES_SRC)
set(MTF_SRC ${MTF_SRC} ${MTF_UTILITIES_SRC})
set(MTF_INCLUDE_DIRS ${MTF_INCLUDE_DIRS} Utilities/include)
