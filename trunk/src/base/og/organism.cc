// 
//
// Author: Wenguo Liu
// Date: 11/2011
//
//
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include <string.h>
#include "organism.hh"

int OrganismNode::id_index = 0;
char side_names[SIDE_COUNT]= {'F','R','B','L'};
char robottype_names[ROBOTTYPE_COUNT] = {'0','K','S','A'};

double OrganismNode::Geom::geom_size[ROBOTTYPE_COUNT][3] = {
    {0, 0, 0},
    {1 * unit, 1 * unit, 1 * unit},
    {1 * unit, 1 * unit, 1 * unit},
    {1 * unit, 3 * unit, 1 * unit}};


OrganismNode::Geom::Geom():px(0),py(0),pa(0),width(60),height(60)
{
    parent = NULL;
}

OrganismNode::Geom::Geom(const OrganismNode::Geom &c)
{
    parent = c.parent;
    px = c.px;
    py = c.py;
    pa = c.pa;
    width = c.width;
    height = c.height;
}

double OrganismNode::Geom::Distance(const OrganismNode::Geom& c)
{
    return sqrt((px - c.px)*(px - c.px) + (py - c.py)*(py - c.py));
}

OrganismNode::OrganismNode(const robot_type& t)
{
    id = OrganismNode::id_index++;
    type = t;
    for(int i=0; i<SIDE_COUNT; i++)
    {
        connection[i]=NULL;
        depth[i]=0;
        order[i]=0;
    }
    memset(visited, 0, SIDE_COUNT * sizeof(visited_status));

    geom.width = OrganismNode::Geom::geom_size[t][0];
    geom.height =  OrganismNode::Geom::geom_size[t][1];
    geom.length =  OrganismNode::Geom::geom_size[t][2];
    geom.parent = this;
}

OrganismNode::OrganismNode(int index, const robot_type&t)
{
    id = index;
    type = t;
    for(int i=0; i<SIDE_COUNT; i++)
    {
        connection[i]=NULL;
        depth[i]=0;
        order[i]=0;
    }
    memset(visited, 0, SIDE_COUNT * sizeof(visited_status));

    geom.width =  OrganismNode::Geom::geom_size[t][0];
    geom.height =  OrganismNode::Geom::geom_size[t][1];
    geom.length =  OrganismNode::Geom::geom_size[t][2];
    geom.parent = this;
}

void OrganismNode::ResetStatus()
{
    memset(visited, 0, SIDE_COUNT * sizeof(visited_status));
}

void OrganismNode::GetNewGeomFromParent(const OrganismNode& parent, const robot_side& parentSide, const robot_side& mySide)
{
    const float rotation_angle[SIDE_COUNT]={0,-90, 180, 90};

    // distance of two units depends on the docking sides
    float dist =   (fabs (parent.geom.width * cos(DTOR(rotation_angle[parentSide])) +
                    parent.geom.height * sin(DTOR(rotation_angle[parentSide]))) +
                    fabs( geom.width * cos(DTOR(rotation_angle[mySide])) +
                    geom.height * sin(DTOR(rotation_angle[mySide])))) /2.0;

    float angle = parent.geom.pa + rotation_angle[parentSide];
    geom.px = parent.geom.px + dist * cos (DTOR(angle));
    geom.py = parent.geom.py + dist * sin (DTOR(angle));
    geom.pa = angle - rotation_angle[mySide] + 180;
}



bool OrganismNode::FindConnectionSide(robot_side &side, const OrganismNode &node)
{
    for(int i=0;i<SIDE_COUNT;i++)
    {
        if(connection[i] == &node)
        {
            side = robot_side(i);
            return true;
        }
    }

    return false;
}

/////////////////////////////Organism///////////////////////////////
Organism::Organism()
{
    name = strdup("Unknow");
}

Organism::~Organism()
{
    free(name);
}

OrganismNode* Organism::Insert(OrganismNode *parent, OrganismNode *child, const robot_side& parentSide, const robot_side& childSide)
{
    if(parent && child)
    {
        if(parent->connection[parentSide]!=NULL)
        {
            std::cout<<"\033[1;31mWARNING! two nodes are connected to the same parent\033[0m"<<std::endl;
        }
        parent->connection[parentSide] = child;
        child->connection[childSide] = parent;
        child->GetNewGeomFromParent(*parent, parentSide, childSide);
    }

    if(child)
    {
        nodeList.push_back(child);
    }

    return child;
}


void Organism::DFSTraversal(OrganismNode &node, OrganismSequence &seq)
{
    robot_side s;

    for(int i=0;i<SIDE_COUNT;i++)
    {
        if(!node.visited[i].status && node.connection[i]!=NULL)
        {
          //  node.visited[i] = true;
            if(node.connection[i]->FindConnectionSide(s, node))
            {
                node.connection[i]->visited[s].status = true;
                OrganismSequence::Symbol symbol(node.type, robot_side(i), node.connection[i]->type, s);
                seq.encoded_seq.push_back(symbol);
            }
            DFSTraversal(*node.connection[i], seq);
        }
    }
    OrganismSequence::Symbol symbol_tail(0);
    seq.encoded_seq.push_back(symbol_tail);
}

void Organism::DFSTraversal(OrganismNode &node, std::vector<OrganismNode*>& orderedNodeList)
{
    robot_side s;
    orderedNodeList.push_back(&node);
    for(int i=0;i<SIDE_COUNT;i++)
    {
        if(!node.visited[i].status && node.connection[i]!=NULL)
        {
            if(node.connection[i]->FindConnectionSide(s, node))
            {
                node.connection[i]->visited[s].status = true;
                node.connection[i]->visited[s].parent_side = (robot_side)i;
            }
            DFSTraversal(*node.connection[i], orderedNodeList);
        }

    }

    //fill in order[i]
    for(int i=0;i<SIDE_COUNT;i++)
    {
        if(!node.visited[i].status && node.connection[i])
        {
            unsigned int depth = 0;
            for(int j=0;j<SIDE_COUNT;j++)
            {
                if(node.connection[i]->depth[j] > depth)
                    depth = node.connection[i]->depth[j];
                node.order[i] += node.connection[i]->order[j];
            }
            node.order[i] +=1;
            node.depth[i] = depth + 1;
        }
    }

}

void Organism::ResetStatus()
{
    std::vector<OrganismNode*>::iterator it;
    for(it=nodeList.begin();it!=nodeList.end();it++)
        (*it)->ResetStatus();
}

void Organism::GraphToSequence(OrganismNode& node, OrganismSequence& seq)
{
    //clear sequence
    seq.Clear();

    int count = 0;
    for(int i=0;i< SIDE_COUNT;i++)
    {
        if(node.connection[i])
            count++;
    }
    // a single node
    if(count == 0) 
    {
        seq.encoded_seq.push_back(OrganismSequence::Symbol(node.type, FRONT, ROBOT_NONE,FRONT));
        seq.encoded_seq.push_back(OrganismSequence::Symbol(0));

    }
    else
    {
        DFSTraversal(node, seq);
        seq.encoded_seq.pop_back(); //get rid of the last empty 0 inserted automatically
        node.seq = &seq;
    }
}

void Organism::GraphToSequence(OrganismSequence & seq)
{
    if(nodeList.empty())
        seq.Clear();
    else
    {
        ResetStatus();
        Organism::GraphToSequence(*nodeList[0], seq);
        ResetStatus(); //reset twice in case 
    }
}

void Organism::GraphToSequence()
{
    /*
    //clean sequence list first
    while(!seqList.empty())
    {
    OrganismSequence *seq = seqList.back();
    delete seq;
    seqList.pop_back();
    }
    */

    std::vector<OrganismNode*>::iterator it;
    for(it=nodeList.begin();it!=nodeList.end();it++)
    {
        OrganismSequence *seq = new OrganismSequence();
        ResetStatus();
        Organism::GraphToSequence(**it, *seq);
        ResetStatus(); //reset twice in case 
        seqList.push_back(seq);
    }

}

void Organism::Scan()
{
    ResetStatus(); //reset first in case some flags are set somewhere else
    if(!nodeList.empty())
    {
        std::vector<OrganismNode*> orderedNodeList;
        Organism::DFSTraversal(*nodeList[0], orderedNodeList);
        for(unsigned int i=0;i<orderedNodeList.size();i++)
        {
            int known_order_sum = 0;
            for(int j=0;j<SIDE_COUNT;j++)
            {
                //find which side is visited from parent node during traversal
                if(orderedNodeList[i]->visited[j].status)
                {
                    unsigned int depth = 0;
                    for(int k=0;k<SIDE_COUNT;k++)
                    {
                        if( k!= orderedNodeList[i]->visited[j].parent_side)
                        {
                            orderedNodeList[i]->order[j] += orderedNodeList[i]->connection[j]->order[k];
                            //find the depth
                            if(orderedNodeList[i]->connection[j]->depth[k] >depth)
                                depth = orderedNodeList[i]->connection[j]->depth[k];
                        }
                    }
                    orderedNodeList[i]->order[j] += 1;
                    orderedNodeList[i]->depth[j] = depth + 1;
                    //no need to search again since there should be only one parental edge
                    break;
                }
            }
        }
    }
    ResetStatus(); 
}

rt_status Organism::SequenceToGraph(const OrganismSequence& og_seq, Organism& og)
{
    //scan first
    std::vector<OrganismSequence::Element> eList;
    std::vector<unsigned int> edges;
    rt_status ret = OrganismSequence::Scan(og_seq, eList, edges);
    if(ret.status >= RT_ERROR)
    {
        std::cout<<"Error in parsing encoded sequence"<<std::endl;
        return ret;
    }

    OrganismNode * node = og.Insert(NULL, new OrganismNode((robot_type)og_seq.encoded_seq[0].type1), FRONT, BACK);
    OGSeqTraversal(og_seq, og, node);

    ret.status = RT_OK;
    return ret;
}

void Organism::OGSeqTraversal(const OrganismSequence &og_seq, Organism& og, OrganismNode* parent)
{
    //fill branches
    // static int index = 0;
    std::vector<OrganismSequence> branches;
    OrganismSequence::fillBranches(og_seq, branches);
    for(unsigned int i=0;i<branches.size();i++)
    {
        if(branches[i].encoded_seq.size()>0)
        {
            //create new node
            OrganismNode * node = og.Insert(parent, new OrganismNode((robot_type)branches[i].encoded_seq[0].type2),(robot_side)branches[i].encoded_seq[0].side1, (robot_side)branches[i].encoded_seq[0].side2);
            //remove the parent node
            std::vector<OrganismSequence::Symbol> s;
            s.assign(branches[i].encoded_seq.begin() + 1, branches[i].encoded_seq.end() - 1);
            OGSeqTraversal(OrganismSequence(s), og, node);
        }
    }
}


bool Organism::Valid()
{
    for(unsigned int i=0;i<nodeList.size();i++)
    {
        for(unsigned int j=i+1;j<nodeList.size();j++)
        {
            bool connected = false;
            for(int k=0;k<SIDE_COUNT;k++)
            {
                if(nodeList[i]->connection[k] == nodeList[j])
                {
                    connected = true;
                }
            }

            if(!connected &&nodeList[i]->geom.Distance(nodeList[j]->geom) < 1.2*unit) //TODO: at least one robot away
            {
                std::cout<<"Distance:"<<nodeList[i]->geom.Distance(nodeList[j]->geom)<<std::endl;
                return false;
            }
        }
    }

    //note an empty organism is valid
    return true;
}

void Organism::Clear()
{

}

/////////////////////////////////////////////////////////////////////

//debuging functions
//
std::ostream & operator<<(std::ostream& os, const OrganismNode::Geom& p)
{
    os<<"("<<p.px<<", "<<p.py<<", "<<p.pa<<")";
    return os;
}

std::ostream & operator<<(std::ostream& os, const OrganismNode& node)
{
    os<<"node "<<node.id<<" : T-("<<robottype_names[node.type]<<"), D-(";
    for(int i=0;i<SIDE_COUNT;i++)
    {
        os<<node.depth[i]<<"|";
    }
    os<<"), O-(";
    for(int i=0;i<SIDE_COUNT;i++)
    {
        os<<node.order[i]<<"|";
    }
    os<<"),\tC-(";
    for(int i=0;i<SIDE_COUNT;i++)
    {
        if(node.connection[i]!=NULL)
        {
            os<<node.connection[i]->id<<"|";
            bool found=false;
            int j=0;
            for(j=0;j<SIDE_COUNT;j++)
            {
                if(node.connection[i]->connection[j] == &node)
                {
                    found = true;
                    break;
                }
            }
            if(found)
                os<<side_names[j];
            else
                os<<'-';
        }
        else
            os<<'-';

        os<<"\t";
    }
    os<<")";
    return os;
}

std::ostream& operator<<(std::ostream& os, const OrganismSequence::Symbol& sym)
{ 
    if(sym.type1!=0 || sym.type2!=0)
        //os<<side_names[sym.side1]<<side_names[sym.side2];
        os<<robottype_names[sym.type1]<<side_names[sym.side1]<<robottype_names[sym.type2]<<side_names[sym.side2];
    else
        //os<<"00";
        os<<"0000";
    return os;
}

std::ostream& operator<<(std::ostream& os, const OrganismSequence::Element& ele)
{ 
    os<<ele.symbol<<"("<<(int)ele.symbol.data<<") : "<<ele.pair_pos;
    return os;
}


std::ostream& operator<<(std::ostream& os, const OrganismSequence& seq)
{
    if(!seq.encoded_seq.empty())
    {
        for(unsigned int i=0;i<seq.encoded_seq.size();i++)
            os<<seq.encoded_seq[i];
    }
    return os;
}

std::ostream& operator<<(std::ostream& os, const Organism& og)
{
    os<<"Organism "<<og.name<<" with "<<og.nodeList.size()<<" nodes"<<std::endl;
    if(!og.nodeList.empty())
    {
        for(unsigned int i=0;i<og.nodeList.size();i++)
            os << *og.nodeList[i]<<std::endl;
    }
    return os;
}
