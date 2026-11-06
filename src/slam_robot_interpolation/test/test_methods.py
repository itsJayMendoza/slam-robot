import numpy as np
import pytest

from slam_robot_interpolation.methods import idw, ordinary_kriging, predict
from conftest import field


def test_idw_is_exact_at_samples(lattice):
    xy, z = lattice
    np.testing.assert_allclose(idw(xy, z, xy), z)


def test_idw_stays_within_sample_range(lattice):
    xy, z = lattice
    q = np.random.default_rng(1).uniform(-3, 3, (200, 2))
    p = idw(xy, z, q)
    assert p.min() >= z.min() - 1e-9 and p.max() <= z.max() + 1e-9


def test_idw_power_changes_result(lattice):
    xy, z = lattice
    q = np.array([[0.4, 0.3]])
    assert abs(idw(xy, z, q, power=1)[0] - idw(xy, z, q, power=4)[0]) > 0.01


def test_idw_k_nearest(lattice):
    xy, z = lattice
    q = np.array([[0.4, 0.3]])
    assert idw(xy, z, q, k=4)[0] != pytest.approx(idw(xy, z, q)[0])


def test_idw_rejects_bad_input(lattice):
    xy, z = lattice
    with pytest.raises(ValueError):
        idw(xy, z, xy, power=0)
    with pytest.raises(ValueError):
        idw(xy[:0], z[:0], xy)


@pytest.mark.parametrize("model", ["spherical", "exponential", "gaussian", "linear", "power"])
def test_kriging_models_run_and_are_accurate(lattice, model):
    xy, z = lattice
    q = np.random.default_rng(2).uniform(-2.5, 2.5, (100, 2))
    pred, var = ordinary_kriging(xy, z, q, variogram_model=model)
    assert pred.shape == var.shape == (100,)
    assert np.all(var >= 0)
    assert np.sqrt(np.mean((pred - field(q)) ** 2)) < 3.0


def test_kriging_beats_idw_on_smooth_field(lattice):
    xy, z = lattice
    q = np.random.default_rng(3).uniform(-2.5, 2.5, (300, 2))
    err_k = np.sqrt(np.mean((ordinary_kriging(xy, z, q)[0] - field(q)) ** 2))
    err_i = np.sqrt(np.mean((idw(xy, z, q) - field(q)) ** 2))
    assert err_k < err_i


def test_kriging_variance_grows_away_from_samples(lattice):
    xy, z = lattice
    near = np.array([[0.0, 0.0]])        # on a sample
    far = np.array([[9.0, 9.0]])         # well outside the surveyed area
    assert ordinary_kriging(xy, z, far)[1][0] > ordinary_kriging(xy, z, near)[1][0]


def test_kriging_needs_enough_samples_and_valid_model(lattice):
    xy, z = lattice
    with pytest.raises(ValueError):
        ordinary_kriging(xy[:3], z[:3], xy)
    with pytest.raises(ValueError):
        ordinary_kriging(xy, z, xy, variogram_model="nope")


def test_kriging_constant_field(lattice):
    xy, _ = lattice
    pred, var = ordinary_kriging(xy, np.full(len(xy), -55.0), xy[:5])
    assert np.all(pred == -55.0) and np.all(var == 0)


def test_predict_dispatch(lattice):
    xy, z = lattice
    p, v = predict("idw", xy, z, xy[:3])
    assert v is None
    p, v = predict("kriging", xy, z, xy[:3])
    assert v is not None
    with pytest.raises(ValueError):
        predict("nope", xy, z, xy)
