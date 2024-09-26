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

# DOA peak estimate direction
# Zero vector means absence of estimate.
geometry_msgs/Vector3 doa

# DOA raw data: direction (azimuth, colatitude) and corresponding intensity (values)
# * No DOA data
#   - all arrays are empty
# * 2D DOA data 
#   - colatitude is empty
#   - azimuth and values arrays have the same size
#   - direction (cos(azimuth[i]), sin(azimuth[i]), 0) have intensity values[i]
# * 3D DOA data with irregular grid 
#   - azimuth, colatitude and values arrays have the same size
#   - direction (cos(azimuth[i])*sin(colatitude[i]), sin(azimuth[i])*sin(colatitude[i]), cos(colatitude[i])) have intensity values[i]
# * 3D DOA data with regular grid 
#   - azimuth and colatitude are ordered, size(values) = size(azimuth) * size(colatitude)
#   - direction (cos(azimuth[k])*sin(colatitude[l]), sin(azimuth[k])*sin(colatitude[l]), cos(colatitude[l])) have intensity values[l*size(azimuth) + k]
float32[] doa_azimuth
float32[] doa_colatitude
float32[] doa_values

# sound_flag constants
uint16 SOUND_DETECTING = 1
uint16 SPEECH_DETECTING = 2
uint16 SPEECH_DECODED = 4
