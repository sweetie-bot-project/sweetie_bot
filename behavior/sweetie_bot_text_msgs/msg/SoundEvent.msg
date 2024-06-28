std_msgs/Header header
# status
uint16 sound_flags

# speech decode results
# must present if SPEECH_DECODED flag is set
string text
string language

# intensity analisys: genearl intensity, how intensity corresponds to speech 
float64 intensity
float64 speech_probability

# DOA peak estimate
geometry_msgs/Vector3 doa

# DOA raw data (azimuth and cloatitiyude munt be orderd)
# variant 1: azimuth and colatitude are ordered, size(values) = size(azimuth) * size(colatitude)
#            direction  (azimuth[k], colatitude[l]) have intensity values[l*size(azimuth) + k]
# variant 2: azimuth, colatitude and values have the same size
#            direction  (azimuth[k], colatitude[k]) have intensity values[k]
# variant 3: all arrays are empty
float32[] doa_azimuth
float32[] doa_colatitude
float32[] doa_values

# sound_flag constants
uint16 SOUND_DETECTING = 1
uint16 SPEECH_DETECTING = 2
uint16 SPEECH_DECODED = 4
