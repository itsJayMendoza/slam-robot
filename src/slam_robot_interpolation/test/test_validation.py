import numpy as np
import pytest

from slam_robot_interpolation.validation import (Zone, cross_validate, holdout_split,
                                                  inaccessible_zone_test, kfold_indices,
                                                  metrics, thinning_experiment)
from conftest import field


def test_metrics():
    m = metrics([-50, -60, -70], [-52, -58, -70])
    assert m["rmse"] == pytest.approx(np.sqrt((4 + 4 + 0) / 3))
    assert m["mae"] == pytest.approx(4 / 3)
    assert m["bias"] == pytest.approx(0.0)
    assert metrics([-50], [-55])["bias"] == pytest.approx(5.0)   # over-estimate is positive
    assert np.isnan(metrics([], [])["rmse"])


def test_holdout_split_sizes_and_disjoint():
    train, test = holdout_split(100, 0.2, seed=1)
    assert len(test) == 20 and len(train) == 80
    assert not set(train) & set(test)
    assert sorted(np.concatenate([train, test])) == list(range(100))
    t2, s2 = holdout_split(100, 0.2, seed=1)
    assert (t2 == train).all() and (s2 == test).all()          # reproducible
    assert not (holdout_split(100, 0.2, seed=2)[1] == test).all()


def test_holdout_validates_fraction():
    with pytest.raises(ValueError):
        holdout_split(10, 0.0)


def test_kfold_covers_every_sample_exactly_once_as_test():
    folds = kfold_indices(23, 5, seed=0)
    assert len(folds) == 5
    all_test = np.concatenate([t for _, t in folds])
    assert sorted(all_test) == list(range(23))
    for train, test in folds:
        assert not set(train) & set(test) and len(train) + len(test) == 23
    with pytest.raises(ValueError):
        kfold_indices(5, 1)


def test_cross_validation_accuracy_on_smooth_field(lattice):
    xy, z = lattice
    k = cross_validate("kriging", xy, z, k=5)
    i = cross_validate("idw", xy, z, k=5)
    assert k["n"] == i["n"] == len(z)
    assert k["rmse"] < 3.5            # the Experiment 5 target
    assert k["rmse"] < i["rmse"]


def test_zone_geometry():
    r = Zone.rect(0, 0, 2, 2)
    d = r.distance_outside(np.array([[1, 1], [3, 1], [3, 3], [-1, 1]]))
    np.testing.assert_allclose(d, [0, 1, np.sqrt(2), 1])
    c = Zone.circle(0, 0, 1)
    np.testing.assert_allclose(c.distance_outside(np.array([[0.5, 0], [3, 0]])), [0, 2])


@pytest.fixture
def dense():
    g = np.arange(-3.0, 3.01, 0.5)
    xy = np.array([(x, y) for y in g for x in g])
    return xy, field(xy)


def test_zone_test_hides_zone_samples_and_thins_ring(dense):
    xy, z = dense
    zone = Zone.rect(-1, -1, 1, 1)
    full = inaccessible_zone_test("kriging", xy, z, zone, keep_frac=1.0)
    half = inaccessible_zone_test("kriging", xy, z, zone, keep_frac=0.5)
    quarter = inaccessible_zone_test("kriging", xy, z, zone, keep_frac=0.25)
    assert full["n"] == half["n"] == quarter["n"] > 0          # same hidden samples scored
    assert full["ring_kept"] == full["ring_total"]
    assert quarter["ring_kept"] < half["ring_kept"] < full["ring_kept"]
    assert quarter["n_train"] < half["n_train"] < full["n_train"]
    for r in (full, half, quarter):
        assert np.isfinite(r["rmse"])
    assert full["rmse"] < 4.0                                    # Experiment 6 target


def test_thinning_experiment_returns_each_fraction(dense):
    xy, z = dense
    res = thinning_experiment("idw", xy, z, Zone.circle(0, 0, 1.2))
    assert list(res) == [1.0, 0.75, 0.5, 0.25]


def test_zone_test_errors(dense):
    xy, z = dense
    with pytest.raises(ValueError):
        inaccessible_zone_test("idw", xy, z, Zone.rect(50, 50, 51, 51))   # nothing inside
    with pytest.raises(ValueError):
        inaccessible_zone_test("idw", xy, z, Zone.rect(-1, -1, 1, 1), keep_frac=0)
