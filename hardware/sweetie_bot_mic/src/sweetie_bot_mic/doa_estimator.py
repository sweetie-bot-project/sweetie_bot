import numpy as np
import PyKDL
import rospy

import pyroomacoustics as pra

class DOAEstimator:
    def __init__(self, **params):
        #
        # basic parameters
        #
        sample_rate = params.get("sample_rate") 
        if sample_rate is None:
            raise RuntimeError(f'DOA estimator: sample_rate is not specified.')
        doa_algorithm = params.get("algorithm", 'none') 
        if doa_algorithm not in pra.doa.algorithms:
            raise RuntimeError(f'DOA estimator: unknown algorithms {doa_algorithm}')
        mic_coords = params.get("mic_coords")
        if not isinstance(mic_coords, list) or any(not isinstance(c, (float, int)) for c in mic_coords) or len(mic_coords) % 3 != 0:
            raise RuntimeError('DOA estimator: \'mic_coords\' parameter must be specified and be list of float of format [ x1, y1, z1, x2, y2, ... xn, yn, zn ]')
        mic_coords = np.reshape(mic_coords, (len(mic_coords)//3, 3)).T
        self.mic_channels = params.get("mic_channels")
        if not isinstance(self.mic_channels, list) or any(not isinstance(c, int) for c in self.mic_channels) or mic_coords.shape[1] != len(self.mic_channels):
            raise RuntimeError('DOA estimator: \'mic_channels\' parameter must be specified and be list of int, it size must be equal to size of mic_coords divided by two.')
        nfft = params.get("nfft", 256)
        if not isinstance(nfft, int) or nfft < 0 or 2**int(np.log2(nfft)) != nfft:
            raise RuntimeError('DOA estimator: \'nfft\' parameter must be power of 2.')
        freq_range = params.get("freq_range", [80.0, 2000.0])
        if not isinstance(freq_range, list) or len(freq_range) != 2 or any(not isinstance(freq, (int, float)) for freq in freq_range):
            raise RuntimeError('DOA estimator: \'freq_range\' must be list with two frequencies: [min_freq, max_freq].')
        if freq_range[0] >= freq_range[1] or freq_range[0] < 0.0  or freq_range[1] > sample_rate/2.0:
            raise RuntimeError('DOA estimator: incorrect \'freq_range\' [min_freq, max_freq]: the following condition must hold 0.0 <= min_freq <= max_freq <= sample_rate/2.')
        self.freq_bin_range = np.round( np.array(freq_range) / (sample_rate/2.0) * nfft ).astype(np.int32)
        mode = params.get("mode", 'far')
        if not isinstance(mode, str):
            raise RuntimeError("DOA estimator: 'mode' parameter must be string.")
        #
        # grid construction
        #
        grid = params.get("grid_type", None)
        if grid in ('2d_circle', '3d_sphere'):
            # get grid size paramter
            colatitude = None
            azimuth = None
            if grid.startswith('2d'):
                dim = 2
            else:
                dim = 3
            n_grid = params.get("grid_size", 32 if dim == 2 else 32*16)
            if not isinstance(n_grid, int) or n_grid <= 0:
                raise RuntimeError("DOA estimator: 'grid_size' parameter must be positive integer.")
        elif grid in ('2d_linspace', '3d_linspace'):
            # interpret colatitude and azimuth as linspace specification
            n_grid = None
            azimuth = params.get("azimuth", [-180, 180, 32])
            colatitude = params.get("colatitude", None)
            try:
                azimuth = np.deg2rad( np.linspace(azimuth[0], azimuth[1], int(azimuth[2])) )
                if colatitude is not None:
                    colatitude = np.deg2rad( np.linspace(colatitude[0], colatitude[1], int(colatitude[2])) )
            except (TypeError, ValueError, IndexError):
                raise RuntimeError("DOA estimator: 'azimuth' and 'colatitude' paramters must be lists in format [start_angle, stop_angel, N]. 'azimuth' paramter must be specified.")
            # 2d or 3d
            if grid.startswith('2d'):
                dim = 2
                colatitude = None
            else:
                dim = 3
                if colatitude is None:
                    raise RuntimeError("DOA estimator: 'colatitude' parameter must be specified for '3d_lispace' grid mode.")
        else:
            raise RuntimeError("DOA estimator: 'grid_type' parameter must be specified and be string with one of following value: '2d_circle', '3d_sphere', '2d_linspace', '3d_linspace'.")
        #
        # create estimator 
        #
        alg = pra.doa.algorithms[doa_algorithm]
        self.doa_estimator = alg(L = mic_coords, fs = sample_rate, nfft = nfft, num_src = 1, mode = mode, dim = dim, n_grid = n_grid, azimuth = azimuth, colatitude = colatitude)
        self.stft = pra.transform.STFT(N = nfft, hop = nfft // 2, channels = len(self.mic_channels))
        # log info
        rospy.loginfo(f"DOA estimator '{doa_algorithm}': nfft {nfft}, freq range {freq_range}, grid type '{grid}', grid size {self.doa_estimator.grid.n_points}")

    @property
    def dim(self):
        return self.doa_estimator.dim

    @property
    def grid_azimuth(self):
        return self.doa_estimator.grid.azimuth

    @property
    def grid_colatitude(self):
        if self.doa_estimator.dim == 2:
            return []
        else:
            return self.doa_estimator.grid.colatitude

    @property
    def grid_values(self):
        return self.doa_estimator.grid.values

    def prepare_buffers(self, sound_event):
        # add grid paramters to SoundEvent
        sound_event.doa_azimuth = self.doa_estimator.grid.azimuth

    def update(self, data):
        # perform fft on raw mic channels
        X = self.stft.analysis(data[:, self.mic_channels].astype(np.float32))
        # estimate direction
        self.doa_estimator.locate_sources(np.swapaxes(X, 0, 2), freq_bins=np.arange(*self.freq_bin_range)) 
        # calculate peak direction
        sources_azimuth = self.doa_estimator.azimuth_recon
        sources_colatitude = self.doa_estimator.colatitude_recon
        if sources_azimuth is not None and len(sources_azimuth) > 0:
            azimuth = sources_azimuth[0]
        else:
            azimuth = 0.0
        if sources_colatitude is not None and len(sources_colatitude) > 0:
            colatitude = sources_colatitude[0]
        else:
            colatitude = np.pi/2.0
        doa_direction =  PyKDL.Vector(np.cos(azimuth)*np.sin(colatitude), np.sin(azimuth)*np.sin(colatitude), np.cos(colatitude))
        # copy raw DOA data
        doa_values_sum = np.sum(self.doa_estimator.grid.values)
        doa_values = self.doa_estimator.grid.values / doa_values_sum
        return doa_direction, doa_values

