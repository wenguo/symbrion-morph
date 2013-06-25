// 
//
// Author: Wenguo Liu
// Date: 11/2011
//
//
#include <stdio.h>
#include <vector>
#include <ga/ga.h>
#include "canvas_gl.hh"
#include "organism.hh"
#include "GASequenceGenome.hh"

std::vector<fptr> testFunctionList;

void testGenerateOrganism1(Organism * og);
void testGenerateOrganism2(Organism * og);
void* testMergeSequence(void *);
void* testDefinition(void *);
void* testSplitSequence(void *);
void* testCrossOver(void *);
void* testMutation(void *);
void* testInit(void *);
void* testGA(void *);

Canvas * canvas;

int main(int argc, char **argv)
{

    Organism *og = new Organism;
    testGenerateOrganism2(og);
    std::cout<<*og<<std::endl;
    OrganismSequence seq;

    Organism::GraphToSequence(*(og->NodeList()[0]), seq);
    std::cout<<seq<<std::endl;

    //save pointers for all test function
    //using keyboard a b c d e f to choose one 
    testFunctionList.push_back(testInit);
    testFunctionList.push_back(testCrossOver);
    testFunctionList.push_back(testMutation);
    testFunctionList.push_back(testSplitSequence);
    testFunctionList.push_back(testDefinition);
    testFunctionList.push_back(testMergeSequence);
    testFunctionList.push_back(testGA);


    Fl_Window win(800, 600, "OpenGL X");

    canvas = new Canvas(10, 10, win.w()-20, win.h()-20, "Display");
    win.end();
    win.resizable(*canvas);
    win.show();
    
    Fl::run();
    return 1;
}


void testGenerateOrganism1(Organism * og)
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

void testGenerateOrganism2(Organism * og)
{
    OrganismNode * node[10];
    node[0] = og->Insert(NULL, new OrganismNode(ROBOT_KIT), BACK, FRONT);
    node[0]->GetGeom().pa = 90;
    node[1] = og->Insert(node[0], new OrganismNode(ROBOT_KIT), FRONT, BACK);
    node[2] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), FRONT, FRONT);
    node[3] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), RIGHT, LEFT);
    node[4] = og->Insert(node[1], new OrganismNode(ROBOT_KIT), LEFT, BACK);
    node[5] = og->Insert(node[2], new OrganismNode(ROBOT_KIT), BACK, LEFT);

}

void testGenerateOrganism3(Organism * og)
{
    OrganismNode * node[10];
    node[0] = og->Insert(NULL, new OrganismNode(ROBOT_KIT), BACK, FRONT);
    node[0]->GetGeom().pa = 90;
    node[1] = og->Insert(node[0], new OrganismNode(ROBOT_KIT), BACK, FRONT);
    // node[2] = og->Insert(node[0], new OrganismNode(ROBOT_KIT), LEFT, RIGHT);
    // node[3] = og->Insert(node[0], new OrganismNode(ROBOT_KIT), RIGHT, RIGHT);

}

void testGenerateOrganism4(Organism * og)
{
    OrganismNode * node[10];
    node[0] = og->Insert(NULL, new OrganismNode(ROBOT_KIT), BACK, FRONT);
    //node[0]->GetGeom().pa = 90;
    node[1] = og->Insert(node[0], new OrganismNode(ROBOT_SCOUT), BACK, FRONT);
    node[2] = og->Insert(node[1], new OrganismNode(ROBOT_SCOUT), RIGHT, RIGHT);
    node[3] = og->Insert(node[1], new OrganismNode(ROBOT_SCOUT), BACK, RIGHT);
    node[4] = og->Insert(node[2], new OrganismNode(ROBOT_SCOUT), LEFT, RIGHT);
    node[5] = og->Insert(node[1], new OrganismNode(ROBOT_SCOUT), LEFT, RIGHT);

}

void* testMergeSequence(void* ptr)
{
    Canvas *canvas = (Canvas*) ptr;
    std::vector<Organism*>&ogList = canvas->ogList; 
    Organism *og1 = new Organism;
    testGenerateOrganism1(og1);
    Organism *og2 = new Organism;
    testGenerateOrganism3(og2);
    og1->GraphToSequence();
    og2->GraphToSequence();
    ogList.push_back(og1);
    ogList.push_back(og2);

    OrganismSequence og_seq;
    OrganismSequence single_node_seq(ROBOT_SCOUT);
    OrganismSequence single_node_seq1(ROBOT_AW);
    
    rt_status ret;
    do
    {
        unsigned int edge_pos = IRandom(0, og1->SeqList()[4]->Edges()-1);
        ret=OrganismSequence::mergeSequences(*(og1->SeqList()[4]), *(og2->SeqList()[0]), og_seq, edge_pos, false,  LEFT, LEFT);
    }
    while(ret.status >= RT_ERROR);

    if(ret.status == RT_OK)
    {
        Organism *og4 = new Organism;
        Organism::SequenceToGraph(og_seq, *og4);
        ogList.push_back(og4);
    }
    
    canvas->dirty(true);
    return NULL;

}


void* testDefinition(void* ptr)
{
    Canvas *canvas = (Canvas*) ptr;
    std::vector<Organism*>&ogList = canvas->ogList; 
    std::cout<<"test Definition"<<std::endl;
    Organism *og1 = new Organism;
    testGenerateOrganism1(og1);

    //fill in depth information
    og1->Scan();
    std::cout<<"organsim info: "<<std::endl;
    for(int i=0;i<og1->NodeList().size();i++)
    {
        std::cout<<*(og1->NodeList()[i])<<std::endl;
    }


    og1->GraphToSequence();
    ogList.push_back(og1);
    std::cout<<"test sequence: "<<*(og1->SeqList()[0])<<std::endl;

    
    std::vector<OrganismSequence> branches;
    OrganismSequence::fillBranches(*(og1->SeqList()[0]), branches);
    for(int i=0;i<branches.size();i++)
        std::cout<<"branch "<<i<<" : "<<branches[i]<<std::endl;

    OrganismSequence parentSeq;
    OrganismSequence::extractParentSequence(*(og1->SeqList()[0]), parentSeq, 0);
    std::cout<<"parent: "<<parentSeq<<std::endl;

    OrganismSequence childSeq;
    OrganismSequence::extractChildSequence(*(og1->SeqList()[0]), childSeq, 0, false);
    std::cout<<"child: "<<childSeq<<std::endl;

    OrganismSequence::checkNodeConnection(*(og1->SeqList()[0]), 1, false);
    OrganismSequence::checkNodeConnection(*(og1->SeqList()[0]), 1, true);


    std::vector<OrganismSequence::Element> eList;
    std::vector<unsigned int> edges;
    rt_status ret = og1->SeqList()[0]->Scan(eList, edges);

    for(unsigned int i=0;i<eList.size();i++)
        std::cout<<"element "<<i<<": "<<eList[i]<<std::endl;
    for(unsigned int i=0;i<edges.size();i++)
        std::cout<<"edge "<<i<<": "<<edges[i]<<std::endl;
    
    canvas->dirty(true);
    return NULL;
}


void* testSplitSequence(void* ptr)
{
    Canvas *canvas = (Canvas*) ptr;
    std::vector<Organism*>&ogList = canvas->ogList; 
    std::cout<<"test SplitSequence"<<std::endl;

    Organism *og1 = new Organism;
    testGenerateOrganism1(og1);
    og1->GraphToSequence();
    std::cout<<"test sequence (size "<<og1->SeqList()[0]->Size()<<"): "<<*og1->SeqList()[0]<<std::endl;
    ogList.push_back(og1);

    // int j=2;
    // int i=0;
    int count = 0;
    for(int j=0;j<og1->SeqList().size();j++)
        for(int i=0;i<og1->SeqList()[j]->Size()/2;i++)
        {
            OrganismSequence og_seq1, og_seq2, og_seq3;
            rt_status ret=OrganismSequence::splitSequence(*(og1->SeqList()[j]), i, og_seq1, og_seq2);
            OrganismSequence::extractChildSequence(*(og1->SeqList()[j]), og_seq3, 0, false);
            std::cout<<"og_seq3: "<<og_seq3<<std::endl;

            Organism *og2 = new Organism;
            Organism *og3 = new Organism;
            Organism::SequenceToGraph(og_seq1, *og2);
            Organism::SequenceToGraph(og_seq2, *og3);

            //    ogList.push_back(og2);
            //    ogList.push_back(og3);


            std::cout<<"test MergeSequence"<<std::endl;
            OrganismSequence og_seq;
            std::cout<<"og_seq1: "<<og_seq1<<std::endl;
            std::cout<<"og_seq2: "<<og_seq2<<std::endl;
            std::cout<<"edge_pos1: "<<(int)ret.edge_pos1<<"\tside1: "<<(int)ret.side1<<"\tside2: "<<(int)ret.side2<<std::endl;
            ret=OrganismSequence::mergeSequences(og_seq1, og_seq2, og_seq, ret.edge_pos1, ret.status == RT_OK_NOPARENTEDGE,  (robot_side)ret.side1, (robot_side)ret.side2);
            //ret=OrganismSequence::mergeSequences(og_seq1, og_seq2, og_seq, ret.edge_pos1, ret.status != RT_OK_SPLIT_NOCHILDREN,  (robot_side)ret.side1, (robot_side)ret.side2);
            if(ret.status == RT_OK)
            {
                std::cout<<"\033[1;32m"<<j<<" "<<i<<": success in merging sequence\033[0m"<<std::endl;
                Organism *og4 = new Organism;
                Organism::SequenceToGraph(og_seq, *og4);
                ogList.push_back(og4);
            }
            else
            {
                count++;
                std::cout<<"\033[1;31m"<<j<<" "<<i<<": error in merging sequence\033[0m"<<std::endl;
            }
        }

    std::cout<<"failed "<<count<<std::endl;
    
    canvas->dirty(true);
    return NULL;
}


void* testCrossOver(void* ptr)
{
    Canvas *canvas = (Canvas*) ptr;
    std::vector<Organism*>&ogList = canvas->ogList; 
    Organism *og1 = new Organism;
    testGenerateOrganism1(og1);
    Organism *og2 = new Organism;
    testGenerateOrganism4(og2);
    og1->GraphToSequence();
    og2->GraphToSequence();
    ogList.push_back(og1);
    ogList.push_back(og2);

    for(int i=0;i<100;i++)
    {
        OrganismSequence child1, child2;
        OrganismSequence::CrossOver(*og1->SeqList()[0], *og2->SeqList()[0], child1, child2);

        Organism *og3 = new Organism;
        Organism *og4 = new Organism;
        Organism::SequenceToGraph(child1, *og3);
        Organism::SequenceToGraph(child2, *og4);
        ogList.push_back(og3);
        ogList.push_back(og4);
    }

    canvas->dirty(true);
    return NULL;

}

void* testMutation(void* ptr)
{
    Canvas *canvas = (Canvas*) ptr;
    std::vector<Organism*>&ogList = canvas->ogList; 
    Organism *og1 = new Organism;
    testGenerateOrganism1(og1);
    og1->GraphToSequence();
    ogList.push_back(og1);

    for(int i=0;i<100;i++)
    {
        Organism *og2 = new Organism;
        OrganismSequence og_seq_new;
        OrganismSequence::Mutation(*og1->SeqList()[0], og_seq_new);
        Organism::SequenceToGraph(og_seq_new, *og2);
        ogList.push_back(og2);
    }

    canvas->dirty(true);
    return NULL;
}

void* testInit(void* ptr )
{
    Canvas *canvas = (Canvas*) ptr;
    std::vector<Organism*>&ogList = canvas->ogList; 
    int count=0;
    do
    {
        OrganismSequence og_seq;
        OrganismSequence::RandomInit(og_seq, 6);
        Organism* og1 = new Organism;
        Organism::SequenceToGraph(og_seq, *og1);

        if(og1->Valid())
        {
            count++;
            ogList.push_back(og1);
        }
        else
        {
            delete og1;
            std::cout<<"Invalid organism"<<std::endl;
        }
    }
    while(count<100);
    
    canvas->dirty(true);
    return NULL;
}

void* testGA(void* ptr)
{
    Canvas *canvas = (Canvas*) ptr;
    std::vector<Organism*>&ogList = canvas->ogList; 

    GASequenceGenome genome;
    //GAIncrementalGA ga(genome);
   // GASteadyStateGA ga(genome);
    GASimpleGA ga(genome);
    ga.elitist(gaTrue);
    ga.populationSize(30);
    ga.nGenerations(200);
    ga.pCrossover(0.6);
    ga.pMutation(0.1);
    ga.scoreFrequency(1);
    ga.flushFrequency(1);
    ga.scoreFilename("ga.dat");
    ga.initialize();
    int count = 0;
    while(!ga.done())
    {
        //clean drawing list
        pthread_mutex_lock(&canvas->mutex);
        while(!ogList.empty())
        {
            Organism *og = ogList.back();
            delete og;
            ogList.pop_back();
        }
        pthread_mutex_unlock(&canvas->mutex);
        
        std::cout<<"--- generation "<<count++<<" ----"<<std::endl;
        //clean og drawing list
        for(int i=0;i<ga.population().size();i++)
        {
            GASequenceGenome & node = DYN_CAST(GASequenceGenome&, ga.population().individual(i));
            pthread_mutex_lock(&canvas->mutex);
            Organism* og1 = new Organism;
            Organism::SequenceToGraph(node, *og1);
            ogList.push_back(og1);
            pthread_mutex_unlock(&canvas->mutex);
        }
        
        ga.step();
        canvas->dirty(true);
        usleep(110000);
        
//        while(!canvas->step);
        canvas->step = false;
    }
    genome = ga.statistics().bestIndividual();

    pthread_mutex_lock(&canvas->mutex);
    Organism* og1 = new Organism;
    Organism::SequenceToGraph(genome, *og1);
    ogList.push_back(og1);
    pthread_mutex_unlock(&canvas->mutex);
    canvas->dirty(true);

    std::cout << "the best sequence is:\n" << genome << "\n";
    std::cout<<"size:"<<genome.Edges() + 1<<std::endl;
    std::cout << "best of generation data are in '" << ga.scoreFilename() << "'\n";


    return NULL;

}
