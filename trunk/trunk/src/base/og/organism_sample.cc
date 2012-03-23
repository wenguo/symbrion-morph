#include "organism_sample.hh"

void testOrganism1(Organism * og)
{
    OrganismNode * node[12];
    node[0] = og->Insert(NULL, new OrganismNode(ROBOT_SCOUT), BACK, FRONT);
    node[1] = og->Insert(node[0], new OrganismNode(ROBOT_KIT), FRONT, LEFT);
    node[2] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), RIGHT, FRONT);
    node[3] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), BACK, LEFT);
    node[4] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), FRONT, RIGHT);
    node[5] = og->Insert(node[3], new OrganismNode(ROBOT_KIT), RIGHT, LEFT);
    node[6] = og->Insert(node[4], new OrganismNode(ROBOT_KIT), LEFT, RIGHT);
    node[7] = og->Insert(node[5], new OrganismNode(ROBOT_AW), RIGHT, FRONT);
    node[8] = og->Insert(node[6], new OrganismNode(ROBOT_AW), LEFT, FRONT);
    node[9] = og->Insert(node[0], new OrganismNode(ROBOT_AW), BACK, FRONT);
    node[10] = og->Insert(node[2], new OrganismNode(ROBOT_AW), BACK, FRONT);
}

//a crossing with 7 robots
void testOrganism2(Organism * og)
{
    OrganismNode * node[10];
    node[0] = og->Insert(NULL, new OrganismNode(ROBOT_KIT), FRONT);
    node[1] = og->Insert(node[0], new OrganismNode(ROBOT_KIT), BACK);
    node[2] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), BACK);
    node[3] = og->Insert(node[2], new OrganismNode(ROBOT_KIT), BACK);
    node[4] = og->Insert(node[3], new OrganismNode(ROBOT_KIT), BACK);
    node[5] = og->Insert(node[2], new OrganismNode(ROBOT_KIT), LEFT);
    node[6] = og->Insert(node[2], new OrganismNode(ROBOT_KIT), RIGHT);
}

//a snake with 5 robots
void testOrganism3(Organism * og)
{
    OrganismNode * node[10];
    node[0] = og->Insert(NULL, new OrganismNode(ROBOT_KIT), FRONT);
    node[1] = og->Insert(node[0], new OrganismNode(ROBOT_KIT), BACK);
    node[2] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), BACK);
    node[3] = og->Insert(node[2], new OrganismNode(ROBOT_KIT), BACK);
    node[4] = og->Insert(node[3], new OrganismNode(ROBOT_KIT), BACK);
}

//a Z with 8 robots
void testOrganism4(Organism * og)
{
    OrganismNode * node[10];
    node[0] = og->Insert(NULL, new OrganismNode(ROBOT_KIT), FRONT);
    node[1] = og->Insert(node[0], new OrganismNode(ROBOT_KIT), BACK);
    node[2] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), BACK);
    node[3] = og->Insert(node[2], new OrganismNode(ROBOT_KIT), LEFT);
    node[4] = og->Insert(node[3], new OrganismNode(ROBOT_KIT), BACK);
    node[5] = og->Insert(node[4], new OrganismNode(ROBOT_KIT), BACK);
    node[6] = og->Insert(node[5], new OrganismNode(ROBOT_KIT), RIGHT);
    node[7] = og->Insert(node[6], new OrganismNode(ROBOT_KIT), BACK);
}

// KIT -- AW -- KIT -- AW
void RealDemoOrganism_KAKA(Organism * og)
{
    OrganismNode * node[10];
    node[0] = og->Insert(NULL, new OrganismNode(ROBOT_KIT), FRONT);
    node[1] = og->Insert(node[0], new OrganismNode(ROBOT_AW), BACK, FRONT);
    node[2] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), BACK, FRONT);
    node[3] = og->Insert(node[2], new OrganismNode(ROBOT_AW), BACK, FRONT);
}
// AW -- KIT -- AW --KIT
void RealDemoOrganism_AKAK(Organism * og)
{
    OrganismNode * node[10];
    node[0] = og->Insert(NULL, new OrganismNode(ROBOT_AW), BACK);
    node[1] = og->Insert(node[0], new OrganismNode(ROBOT_KIT), FRONT, FRONT);
    node[2] = og->Insert(node[1], new OrganismNode(ROBOT_AW), BACK, FRONT);
    node[3] = og->Insert(node[2], new OrganismNode(ROBOT_KIT), BACK, FRONT);
}

// KIT -- AW -- KIT
void RealDemoOrganism_KAK(Organism * og)
{
    OrganismNode * node[10];
    node[0] = og->Insert(NULL, new OrganismNode(ROBOT_KIT), FRONT);
    node[1] = og->Insert(node[0], new OrganismNode(ROBOT_AW), BACK, FRONT);
    node[2] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), BACK, FRONT);
}
// AW -- KIT -- AW
void RealDemoOrganism_AKA(Organism * og)
{
    OrganismNode * node[10];
    node[0] = og->Insert(NULL, new OrganismNode(ROBOT_AW), FRONT);
    node[1] = og->Insert(node[0], new OrganismNode(ROBOT_KIT), BACK, FRONT);
    node[2] = og->Insert(node[1], new OrganismNode(ROBOT_AW), BACK, FRONT);
}


// KIT -- KIT
void RealDemoOrganism_KK(Organism * og)
{
    OrganismNode * node[10];
    node[0] = og->Insert(NULL, new OrganismNode(ROBOT_KIT), FRONT);
    node[1] = og->Insert(node[0], new OrganismNode(ROBOT_KIT), BACK, FRONT);
}

// KIT -- AW
void RealDemoOrganism_KA(Organism * og)
{
    OrganismNode * node[10];
    node[0] = og->Insert(NULL, new OrganismNode(ROBOT_KIT), FRONT);
    node[1] = og->Insert(node[0], new OrganismNode(ROBOT_AW), BACK, FRONT);
}


