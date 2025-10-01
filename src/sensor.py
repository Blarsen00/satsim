import numpy as np
import matplotlib.pyplot as plt
from typing import Optional, Union


class Sensor:
    """ Generic sensor that takes ground truth, and returns a 
    noise injected signal according to its bias and covariance.
    """

    def __init__(self,
                 cov: Optional[Union[np.ndarray, float]],
                 bias: Optional[Union[np.ndarray, float]]) -> None:

        assert type(cov) == type(bias), f"Both covaraince/std and bias needs to be of the same type: {type(cov)}, {type(bias)}"

        if type(cov) == np.ndarray and type(bias) == np.ndarray:
            assert bias.ndim == 1, "Bias needs to be a 1D vector"
            assert cov.ndim == 2, "Covariance needs to be a 2D matrix"
            assert cov.shape[0] == cov.shape[1], "Covariance matrix needs to be square"
            assert cov.shape[0] == len(bias), f"Bias needs the same number of entries as the number of rows in the covariance matrix: {cov.ndim}, {len(bias)}"

        self.cov = cov
        self.bias = bias


    def inject_noise(self, ground_truth: Optional[Union[np.ndarray, float]]):
        assert isinstance(ground_truth, (float, np.floating)) == isinstance(self.cov, (float, np.floating)) == isinstance(self.bias, (float, np.floating)), f"ground truth needs to be of the same type as the bias and covariance: {isinstance(ground_truth, (float, np.floating))}, {isinstance(self.bias, (float, np.floating))}, {isinstance(self.cov, (float, np.floating))}"

        if isinstance(ground_truth, (float, np.floating)) and isinstance(self.cov, (float, np.floating)) and isinstance(self.bias, (float, np.floating)):
            return np.random.normal(ground_truth, self.cov) + self.bias
        return np.random.multivariate_normal(ground_truth, self.cov) + self.bias


def test_sensor():
    bias = 0.1
    covariance = 2.00
    gyroscope = Sensor(covariance, bias)

    fig = plt.figure()
    ax = fig.add_subplot()

    x = np.arange(0, 100, 0.1)
    y = 10 * np.sin(x)

    y_m = np.zeros_like(y)
    y_m[0] = gyroscope.inject_noise(y[0])

    for i, m in enumerate(y[1:]):
        y_m[i] = y_m[i-1] + gyroscope.inject_noise(m - y_m[i-1])

    ax.plot(x, y_m, linestyle="-", label="Gyroscope estimate")
    ax.plot(x, y, linestyle="--", label="Ground Truth")
    ax.legend()
    ax.grid()
    plt.show()


if __name__ == '__main__':
    test_sensor()
