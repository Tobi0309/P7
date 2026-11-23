#ifndef DSP_GRID_SEARCH_HPP_
#define DSP_GRID_SEARCH_HPP_

#include "dsl/gridcost.h"
#include "dsl/gridsearch.h"

#include <chrono>
#include <optional>
#include <stdexcept>

namespace dsp
{

// Thrown out of DspGridSearch::Plan() when the search runs past its deadline.
class SearchBudgetExceeded : public std::runtime_error
{
public:
    SearchBudgetExceeded() : std::runtime_error("DSP search time budget exceeded") {}
};

// DSL's GridCost with a tighter, still admissible heuristic. Every DSP cell costs at least 1
// (free = 1 + height penalty, unknown = DSP_UNKNOWN, occupied/padded = DSP_OCCUPIED), so every
// edge costs at least (1 + (1 + 1) / 2) * length = 2 * length. DSL's default heuristic is the
// plain distance, which makes D* Lite expand far more cells than it needs to. Real() is unchanged.
class DspGridCost : public dsl::GridCost<3>
{
public:
    double Heur(const dsl::Cell<3> &a, const dsl::Cell<3> &b) const override
    {
        return 2.0 * (a.c - b.c).norm();
    }
};

// Lazily expanded DSL GridSearch: vertices and edges are only created where a search goes,
// instead of for the whole grid up front (expand=true allocates every cell plus 26 edges each).
//
// DSL's own SetCost() cannot be used here: in lazy mode it dereferences a null vertex for any
// cell the search has not reached yet, and when several searches share one Grid it returns
// early as soon as the shared cell already holds the new cost, so only the first search would
// see an update. SyncCellCost() handles both cases.
class DspGridSearch : public dsl::GridSearch<3>
{
public:
    DspGridSearch(const dsl::Grid<3> &grid,
                  const dsl::GridConnectivity<3> &connectivity,
                  const dsl::GridCost<3> &cost)
        : dsl::GridSearch<3>(grid, connectivity, cost, false)
    {
    }

    // Call after writing the new cost into grid.cells[id]. A cell without a vertex needs
    // nothing: its vertex and edges are created from the Grid when the search reaches it.
    void SyncCellCost(int id, double cell_cost)
    {
        CellVertex *vertex = vertexMap[id];
        if (!vertex || vertex->data.cost == cell_cost) {
            return;
        }
        vertex->data.cost = cell_cost;
        for (auto &in : vertex->in) {
            this->ChangeCost(*in.second, this->cost.Real(in.second->from->data, vertex->data));
        }
        for (auto &out : vertex->out) {
            this->ChangeCost(*out.second, this->cost.Real(vertex->data, out.second->to->data));
        }
    }

    bool HasVertex(int id) const
    {
        return vertexMap[id] != nullptr;
    }

    // Plan() throws SearchBudgetExceeded once `deadline` has passed (std::nullopt: no limit).
    // Aborting is safe: DSL's SetGoal() calls Reset(), which clears the open list and every
    // vertex's g/rhs, so the next SetStart()/SetGoal()/Plan() starts from a clean state. The
    // vertices and edges built so far stay valid and are reused.
    void SetDeadline(std::optional<std::chrono::steady_clock::time_point> deadline)
    {
        deadline_ = deadline;
    }

    // Called by DSL once per vertex expansion. The check runs before the base class touches
    // the vertex, so an abort never leaves a half-expanded vertex behind.
    bool Expand(CellVertex &from, bool fwd = true) override
    {
        if (deadline_ && std::chrono::steady_clock::now() > *deadline_) {
            throw SearchBudgetExceeded();
        }
        return dsl::GridSearch<3>::Expand(from, fwd);
    }

private:
    std::optional<std::chrono::steady_clock::time_point> deadline_;
};

}  // namespace dsp

#endif  // DSP_GRID_SEARCH_HPP_
