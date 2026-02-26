idf_component_register(SRCS "main.c" "gpio/spi.c" "radio/sx1262.c" "parse/nmea.c" "gps/neo6m.c"
                    INCLUDE_DIRS "." "gpio" "radio" "parse" "gps")