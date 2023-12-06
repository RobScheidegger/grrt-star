#include <iostream>

#include "graphs/roadmap_vertex.h"
#include "gtest/gtest.h"

#include <memory>
#include "constants.h"
#include "graphs/roadmap.h"
#include "robots/drone/drone.hpp"

using namespace grrt;

TEST(AllPairsShortestPath, Basic) {
    Roadmap roadmap("test", RobotType::DRONE);

    RoadmapVertex::SharedPtr a = roadmap.addVertex("A", std::make_shared<DroneState>(Point(0, 0, 0), 1));
    RoadmapVertex::SharedPtr b = roadmap.addVertex("B", std::make_shared<DroneState>(Point(0, 0, 0), 1));
    RoadmapVertex::SharedPtr c = roadmap.addVertex("C", std::make_shared<DroneState>(Point(0, 0, 0), 1));
    RoadmapVertex::SharedPtr d = roadmap.addVertex("D", std::make_shared<DroneState>(Point(0, 0, 0), 1));
    RoadmapVertex::SharedPtr e = roadmap.addVertex("E", std::make_shared<DroneState>(Point(0, 0, 0), 1));

    roadmap.addDart(a, b, 3);
    roadmap.addDart(a, c, 8);
    roadmap.addDart(a, e, 4);
    roadmap.addDart(b, d, 1);
    roadmap.addDart(b, e, 7);
    roadmap.addDart(c, b, 4);
    roadmap.addDart(d, a, 2);
    roadmap.addDart(d, c, 2);
    roadmap.addDart(e, d, 6);

    roadmap.computeAllPairsShortestPath();

    EXPECT_EQ(roadmap.distances[a->m_id][a->m_id], 0);
    EXPECT_EQ(roadmap.distances[a->m_id][b->m_id], 3);
    EXPECT_EQ(roadmap.distances[a->m_id][c->m_id], 6);
    EXPECT_EQ(roadmap.distances[a->m_id][d->m_id], 4);
    EXPECT_EQ(roadmap.distances[a->m_id][e->m_id], 4);

    EXPECT_EQ(roadmap.distances[b->m_id][a->m_id], 3);
    EXPECT_EQ(roadmap.distances[b->m_id][b->m_id], 0);
    EXPECT_EQ(roadmap.distances[b->m_id][c->m_id], 3);
    EXPECT_EQ(roadmap.distances[b->m_id][d->m_id], 1);
    EXPECT_EQ(roadmap.distances[b->m_id][e->m_id], 7);

    EXPECT_EQ(roadmap.distances[c->m_id][a->m_id], 7);
    EXPECT_EQ(roadmap.distances[c->m_id][b->m_id], 4);
    EXPECT_EQ(roadmap.distances[c->m_id][c->m_id], 0);
    EXPECT_EQ(roadmap.distances[c->m_id][d->m_id], 5);
    EXPECT_EQ(roadmap.distances[c->m_id][e->m_id], 11);

    EXPECT_EQ(roadmap.distances[d->m_id][a->m_id], 2);
    EXPECT_EQ(roadmap.distances[d->m_id][b->m_id], 5);
    EXPECT_EQ(roadmap.distances[d->m_id][c->m_id], 2);
    EXPECT_EQ(roadmap.distances[d->m_id][d->m_id], 0);
    EXPECT_EQ(roadmap.distances[d->m_id][e->m_id], 6);

    EXPECT_EQ(roadmap.distances[e->m_id][a->m_id], 8);
    EXPECT_EQ(roadmap.distances[e->m_id][b->m_id], 11);
    EXPECT_EQ(roadmap.distances[e->m_id][c->m_id], 8);
    EXPECT_EQ(roadmap.distances[e->m_id][d->m_id], 6);
    EXPECT_EQ(roadmap.distances[e->m_id][e->m_id], 0);
}