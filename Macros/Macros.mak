# ----------------------------------------------------------------------------- #
# --------------------------------- Macros --------------------------------- #
# ----------------------------------------------------------------------------- #
MACROS_INCLUDE_DIR = Macros/include
MACROS_HEADER_DIR = ${MACROS_INCLUDE_DIR}/mtf/Macros
MACROS = common register
MACROS_HEADERS =  $(addprefix ${MACROS_HEADER_DIR}/, $(addsuffix .h, ${MACROS}))

MTF_INCLUDE_DIRS += ${MACROS_INCLUDE_DIR}
MTF_HEADERS += ${MACROS_HEADERS}

BASE_HEADERS += ${MACROS_HEADERS}
DIAG_BASE_CLASSES += Macros/common Macros/register