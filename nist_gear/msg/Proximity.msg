# Single reading from a binary proximity sensor that detects the presence of
# objects within its sensing range.
Header header           # timestamp in the header is the time the sensor
                        # returned the sensor reading
bool object_detected    # is there something in the proximity of the sensor?
float32 min_range       # minimum range value [m]
float32 max_range       # maximum range value [m]
