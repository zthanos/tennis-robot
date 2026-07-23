"""Phase 0 fixture catalog contract tests."""

from collection_route_fixtures import ALL_FIXTURES


def test_every_fixture_declares_complete_expected_ball_results():
    for fixture in ALL_FIXTURES:
        snapshot_ids = {ball.ball_id for ball in fixture.snapshot.balls}
        result_ids = {result.ball_id for result in fixture.expected_ball_results}
        assert snapshot_ids == result_ids, fixture.name
        assert len(result_ids) == len(fixture.expected_ball_results), fixture.name


def test_empty_and_unreachable_fixture_statuses_are_explicit():
    by_name = {fixture.name: fixture for fixture in ALL_FIXTURES}
    assert by_name["empty_scan"].expected_planning_status.value == "empty_no_balls"
    assert by_name["all_unreachable"].expected_planning_status.value == "empty_no_feasible_targets"


def test_fixture_catalog_includes_partial_deferred_planning_budget_case():
    by_name = {fixture.name: fixture for fixture in ALL_FIXTURES}
    fixture = by_name["planning_budget_deferred"]
    assert fixture.expected_planning_status.value == "partial"
    assert {result.reason_code.value for result in fixture.expected_ball_results} == {
        "selected",
        "planning_budget",
    }
