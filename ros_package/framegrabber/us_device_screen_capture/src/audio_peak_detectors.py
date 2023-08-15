import numpy as np

'''
Detection methods for biopsy shot detection can be added to this script.
Methods return the frame position of the biopsy shot sound in the given audio array. 
Gradient detection method is taken from the previous IDP project. 
'''


def gradient_method(window_audio_data, peak_frame_interval, gradient_threshold=15000, magnitude_threshold=30000, method="grad_1"):
    """
    :param window_audio_data: numpy array of audio
    :param peak_frame_interval: defined peak interval in terms of the number of frames
    :param magnitude_threshold: magnitude threshold for the biopsy shot sound
    :param gradient_threshold: gradient threshold for the biopsy shot sound
    :param method: "grad_1" for the 1st derivative method, "grad_4" for the 4th derivative method
    :return peak_grad_pos: frame position of the biopsy shot sound, return None if no peak was detected
    """

    gradient = np.gradient(window_audio_data)
    if method == "grad_4":
        for _ in range(0, 3):
            gradient = np.gradient(gradient)

    peak_grad_pos = np.argmax(gradient)
    peak_grad = gradient[peak_grad_pos]

    # It is assumed that the peak sound is the loudest sound in the environment.
    # The rising and decaying times of the peak is around 0.2 seconds,
    # where the peak duration is assumed to be around 1 second.
    start_pos = np.max([peak_grad_pos - peak_frame_interval // 5, 0])
    end_pos = np.min([peak_grad_pos + peak_frame_interval // 5, len(window_audio_data)])
    peak_magnitude = np.max(np.abs(window_audio_data[start_pos:end_pos]))

    if peak_grad > gradient_threshold and peak_magnitude > magnitude_threshold:
        return peak_grad_pos
    else:
        return None


