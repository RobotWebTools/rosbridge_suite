from __future__ import annotations

import unittest
from unittest import mock

from rosapi import params

PARAMS_BY_NODE = {
    "/node_a": ["foo", "bar"],
    "/node_b": ["foo", "baz"],
}


class TestGetParamNamesGlobs(unittest.TestCase):
    def setUp(self) -> None:
        patches = [
            mock.patch.object(params, "get_nodes", return_value=list(PARAMS_BY_NODE)),
            mock.patch.object(
                params, "_get_param_names", side_effect=lambda node: list(PARAMS_BY_NODE[node])
            ),
        ]
        for patch in patches:
            patch.start()
            self.addCleanup(patch.stop)

    def test_no_glob_returns_everything(self) -> None:
        self.assertEqual(params.get_param_names(None), ["foo", "bar", "foo", "baz"])

    def test_empty_glob_denies_everything(self) -> None:
        self.assertEqual(params.get_param_names([]), [])

    def test_glob_filters_names(self) -> None:
        self.assertEqual(params.get_param_names(["ba*"]), ["bar", "baz"])

    def test_glob_without_match_returns_nothing(self) -> None:
        self.assertEqual(params.get_param_names(["nothing*"]), [])

    def test_node_param_names_empty_glob(self) -> None:
        self.assertEqual(params.get_node_param_names("/node_a", []), [])
        self.assertEqual(params.get_node_param_names("/node_a", None), ["foo", "bar"])


if __name__ == "__main__":
    unittest.main()
