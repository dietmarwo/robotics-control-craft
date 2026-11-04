"""In-process prepared evaluation with one native crossing per batch."""

import json
from importlib.resources import files

from ._native import Prepared as _Prepared


def _encode(value):
    return json.dumps(value, sort_keys=True, allow_nan=False, separators=(",", ":"))


class Prepared:
    def __init__(self, model):
        self._native = _Prepared(_encode(model))

    def batch(self, candidates, **budget):
        return json.loads(self._native.batch(_encode(candidates), _encode(budget)))

    def scores(self, candidates, **budget):
        """Return ordered objectives and statuses for untraced candidates."""
        return json.loads(self._native.scores(_encode(candidates), _encode(budget)))

    def score_matrix(self, rows, **budget):
        """Score untraced parameter rows without JSON candidates or records.

        ``rows`` is a list of equal-length lists or a 2-D NumPy array. Returns
        ``(scores, statuses, steps)`` as three lists in row order. Scores are NaN
        for every row whose status is not ``"complete"``.
        """
        if hasattr(rows, "tolist"):
            if getattr(rows, "ndim", 2) != 2:
                raise ValueError("score_matrix expects a 2-D array of parameter rows")
            rows = rows.tolist()
        scores, statuses, steps = self._native.score_matrix(rows, _encode(budget))
        return scores, statuses, steps

    @property
    def model_sha256(self):
        """SHA-256 of the encoded model bytes, as reported in batch results."""
        return self._native.model_sha256

    def evaluate(self, parameters, *, trace=False, **budget):
        return self.batch([{"id": "one", "parameters": parameters, "trace": trace}], **budget)[
            "records"
        ][0]


def load_example():
    """Return the packaged model and a replayable parameter vector."""
    return json.loads(files(__package__).joinpath("example.json").read_text())
