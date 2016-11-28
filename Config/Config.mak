# ---------------------------------------------------------------------------- #
# ---------------------------------- Config ---------------------------------- #
# ---------------------------------------------------------------------------- #
CONFIG_INCLUDE_DIR = Config/include
CONFIG_HEADER_DIR = ${CONFIG_INCLUDE_DIR}/mtf/Config
CONFIG_INCLUDE_FLAGS = -I${CONFIG_INCLUDE_DIR}
MTF_INCLUDE_DIRS += ${CONFIG_INCLUDE_DIR}

CONFIG = parameters datasets
CONFIG_HEADERS =  $(addprefix ${CONFIG_HEADER_DIR}/, $(addsuffix .h, ${CONFIG}))
MTF_HEADERS += ${CONFIG_HEADERS}
