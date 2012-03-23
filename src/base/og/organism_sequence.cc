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
#include <algorithm>
#include "organism.hh"

//TODO: fix compare with eList.end in getBranch and fillBranches

/////////////////////////OrganismSequence
OrganismSequence::Symbol::Symbol(uint8_t sym)
{
    data = sym;
}

void OrganismSequence::Symbol::reBuild(const char str[4])
{
    type1 = getType(str[0]);
    side1 = getSide(str[1]);
    type2 = getType(str[2]);
    side2 = getSide(str[3]);
}


OrganismSequence::Symbol::Symbol(const OrganismSequence::Symbol& s)
{
    data = s.data;
}

OrganismSequence::Symbol::Symbol(const robot_type& t1, const robot_side& s1, const robot_type&t2, const robot_side &s2)
{
    type1 = t1;
    side1 = s1;
    type2 = t2;
    side2 = s2;
}

OrganismSequence::Symbol& OrganismSequence::Symbol::operator=(const OrganismSequence::Symbol& rhs)
{
    if(this!=&rhs)
        data = rhs.data;

    return *this;
}

bool OrganismSequence::Symbol::operator==(const OrganismSequence::Symbol& rhs) const
{
    return data == rhs.data;
}

bool OrganismSequence::Symbol::operator!=(const OrganismSequence::Symbol& rhs) const
{
    return data != rhs.data;
}

uint8_t OrganismSequence::Symbol::getType(const char type)
{
    for(uint8_t i=0;i<ROBOTTYPE_COUNT;i++)
    {
        if(robottype_names[i] == type)
            return i;
    }
    return 0;
}

uint8_t OrganismSequence::Symbol::getSide(const char side)
{
    for(uint8_t i=0;i<SIDE_COUNT;i++)
    {
        if(side_names[i] == side)
            return i;
    }
    return 0;
}

OrganismSequence::Element::Element()
{
}

OrganismSequence::Element::~Element()
{
}

OrganismSequence::Element::Element(const Element& c)
{
    symbol = c.symbol;
    pair_pos = c.pair_pos;
}

OrganismSequence::Element & OrganismSequence::Element::operator=(const Element& rhs)
{
    if(this!=&rhs)
    {
        symbol = rhs.symbol;
        pair_pos = rhs.pair_pos;
    }
    return *this;
}

OrganismSequence::OrganismSequence()
{
}

OrganismSequence::OrganismSequence(const robot_type& t)
{
    encoded_seq.push_back(Symbol(t, robot_side(0), robot_type(0), robot_side(0)));
    encoded_seq.push_back(Symbol(0));
}

OrganismSequence::OrganismSequence(const OrganismSequence& c)
{
    encoded_seq = c.encoded_seq;
}

void OrganismSequence::copy(const OrganismSequence&c)
{
    if(&c == this)
        return;

    encoded_seq = c.encoded_seq;
}

bool OrganismSequence::compare(const OrganismSequence&c)
{
    return *this == c;
}

OrganismSequence::OrganismSequence(const std::vector<Element>& eList)
{
    for(unsigned int i=0;i<eList.size();i++)
        encoded_seq.push_back(eList[i].symbol);
}

OrganismSequence::OrganismSequence(const std::vector<Symbol>& sList)
{
    encoded_seq = sList;
}


OrganismSequence::~OrganismSequence()
{
}

OrganismSequence& OrganismSequence::operator=(const OrganismSequence&rhs)
{
    if(this != &rhs)
    {
        if(!encoded_seq.empty())
            encoded_seq.clear();

        encoded_seq = rhs.encoded_seq;
    }

    return *this;
}

bool OrganismSequence::operator==(const OrganismSequence& rhs) const
{
    if(this == &rhs)
        return true;

    if(encoded_seq.size() != rhs.encoded_seq.size())
        return false;

    for(unsigned int i=0;i<encoded_seq.size();i++)
    {
        if(encoded_seq[i] != rhs.encoded_seq[i])
            return false;
    }

    return true;
}

const OrganismSequence::Symbol OrganismSequence::getSymbol(const unsigned int& pos) const
{
    std::vector<Element> eList;
    std::vector<unsigned int> edges;
    rt_status ret = Scan(eList, edges);
    if(ret.status >= RT_ERROR || pos >= eList.size())
        return Symbol(0);

    if(eList[pos].symbol != Symbol(0))
        return eList[pos].symbol;
    else
        return eList[eList[pos].pair_pos].symbol;
}

const unsigned int OrganismSequence::Edges() const
{
    if(encoded_seq.size()<1)
        return 0;
    else if(encoded_seq[0].type2 == ROBOT_NONE)
        return 0;
    else
        return encoded_seq.size() / 2;
}

rt_status OrganismSequence::Scan(const OrganismSequence& og_seq, std::vector<OrganismSequence::Element>& element_list, std::vector<unsigned int>& edges) 
{
    rt_status ret;
    //simply validation check of encoded_seq
    if(og_seq.encoded_seq.size()==0)
    {
        ret.status = RT_OK_EMPTY_SEQUENCE;
        ret.data = 0;
        return ret;
    }

    if(og_seq.encoded_seq.size() % 2 !=0)
    {
        ret.status = RT_ERROR_PARSE;
        ret.data = 0;
        return ret;
    }

    int count = 0;
    std::vector<Element* > tmp;

    //put every symbols into list
    for (unsigned int i=0; i < og_seq.encoded_seq.size(); i++)
        element_list.push_back(Element(og_seq.encoded_seq[i]));

    //scan for pair_pos of each symbol
    //fill in those non-'0' first
    for (unsigned int i=0; i < og_seq.encoded_seq.size(); i++)
    {
        if (element_list[i].symbol == Symbol(0))
        {
            if(tmp.back())
            {
                tmp.back()->pair_pos = i;
                tmp.pop_back();
                count++;
            }
        }
        else
        {
            if(element_list[i].symbol.type2 != ROBOT_NONE)
                edges.push_back(i);
            tmp.push_back(&(element_list[i]));
        }
    }

    //then fill in those with '0'
    for (unsigned int i=0; i < element_list.size(); i++)
    {
        if (element_list[i].symbol != Symbol(0))
            element_list[element_list[i].pair_pos].pair_pos = i;
    }

    ret.status = RT_OK;
    ret.data = count;
    return ret;
}

rt_status OrganismSequence::Scan(std::vector<OrganismSequence::Element>& element_list, std::vector<unsigned int>& edges) const
{
    return OrganismSequence::Scan(*this, element_list, edges);
}

rt_status OrganismSequence::reBuild(const uint8_t *data, const int size)
{
    //clear 
    encoded_seq.clear();

    //refill
    for(int i=0; i<size; i++)
        encoded_seq.push_back(Symbol(data[i]));

    //checking
    std::vector<Element> eList;
    std::vector<unsigned int> edges;
    return Scan(eList, edges);
}

rt_status OrganismSequence::reBuild(const char *str)
{
    //clear
    encoded_seq.clear();

    int size = strlen(str) / 4;
    for(int i=0;i<size;i++)
    {
        Symbol sym;
        sym.reBuild(str + i * 4);
        encoded_seq.push_back(sym);
    }

    //checking
    std::vector<Element> eList;
    std::vector<unsigned int> edges;
    return Scan(eList, edges);

}

//if error while parsing the sequence, return -1, users need to free memory allocated in branches in this case
rt_status OrganismSequence::fillBranches(const OrganismSequence& og_seq, std::vector<OrganismSequence>& branches)
{
    std::vector<Element> eList;
    std::vector<unsigned int> edges;
    rt_status ret = Scan(og_seq, eList, edges);
    if(ret.status >= RT_ERROR)
        return ret;

    int count=0;

    std::vector<Element>::iterator start, end;
    start = eList.begin();

    while(start!=eList.end())
    {
        end = eList.begin() + start->pair_pos + 1; 
        if(end > eList.end())
        {
            count = -1;
            break;
        }

        //is it a single node representation
        if(start->symbol.type2 != ROBOT_NONE)
        {
            std::vector<Element> e;
            e.assign(start, end);
            branches.push_back(OrganismSequence(e));
            count++;
        }

        start = end;
    }

    ret.status = RT_OK;
    ret.data = count;
    return ret;
}

rt_status OrganismSequence::getBranch(OrganismSequence& branch, const robot_side& side)
{
    std::vector<Element> eList;
    std::vector<unsigned int> edges;
    rt_status ret = Scan(eList, edges);
    if(ret.status >= RT_ERROR)
        return ret;

    std::vector<Element>::iterator start, end;
    start = eList.begin();

    ret.status = RT_OK_EMPTY_BRANCH;
    ret.data = 0;
    while(start!=eList.end())
    {
        end = eList.begin() + start->pair_pos + 1; 
        if(end > eList.end())
            break;

        if( start->symbol.side1 ==side)
        {
            std::vector<Element> e;
            e.assign(start, end);
            branch = OrganismSequence(e);
            ret.status = RT_OK;
            ret.data = e.size();
        }
        start = end;
    }

    return ret;
}


rt_status OrganismSequence::parentEdge(std::vector<Element>& eList, const std::vector<unsigned int>& edges, const unsigned int& edge_pos)
{
    rt_status ret;
    unsigned int pos = edges[edge_pos];
    std::vector<Element>::iterator start, end;
    start = eList.begin() + pos;
    end = eList.begin() + start->pair_pos;
   
    //search forward
    bool found=false;
    while(end < eList.end() - 1)
    {
        //compare
        start = end + 1;
        if( start->symbol == Symbol(0))
        {
            found = true;
            break;
        }
        
        end = eList.begin() +  start->pair_pos;
    }

    if(found)
    {
        for(unsigned i=0;i<edges.size();i++)
        {
            if(edges[i] == start->pair_pos)
            {
                ret.status = RT_OK;
                ret.data = i;
                return ret;
            }
        }
    }


    ret.status = RT_OK_NOPARENTEDGE;
    ret.data = 0;
    return ret;
}

rt_status OrganismSequence::checkNodeConnection(const OrganismSequence& og_seq, const unsigned int& edge_pos, const bool& parentNode)
{
    OrganismSequence sub_seq;
    rt_status extract_status;
    if(parentNode)
        extract_status = extractParentSequence(og_seq, sub_seq, edge_pos);
    else
        extract_status = extractChildSequence(og_seq, sub_seq, edge_pos, true);

   // std::cout<<"checking "<<sub_seq<<std::endl;

    //error to get sub sequence
    if(extract_status.status >= RT_ERROR)
        return extract_status;

    //scan it first
    std::vector<Element> eList;
    std::vector<unsigned int> edges;
    rt_status ret = Scan(sub_seq, eList, edges);
    if(ret.status >= RT_ERROR)
        return ret;

    //parse it
    uint8_t connected_side = 0;
    std::vector<Element>::iterator start = eList.begin();
    std::vector<Element>::iterator end = eList.begin() + eList.size() - 1;
    //check its parent connection, if there is one
    if(extract_status.status != RT_OK_NOGRANDPARENT)
    {
        if(start->pair_pos == eList.size() - 1)
        {
            if(start->symbol.type2 != ROBOT_NONE)
                connected_side |= 1 << (start->symbol.side2);

            start = start + 1;
            end = end - 1;
        }
    }

    //check its child connection
    while(start < end)
    {
        if(start->symbol.type2 != ROBOT_NONE)
            connected_side |= 1<< start->symbol.side1;

        start = eList.begin() + start->pair_pos + 1;
    }

    //std::cout<<"connected side: "<<std::hex<<std::showbase<<(int)connected_side<<std::oct<<std::endl;
    ret.status = RT_OK;
    ret.data = connected_side;
    return ret;
}


bool OrganismSequence::SingleNodeSequence(const OrganismSequence& og_seq)
{
    if(og_seq.encoded_seq.size() == 2 && og_seq.encoded_seq[0].type2 == ROBOT_NONE)
        return true;
    else
        return false;
}


rt_status OrganismSequence::removeChildSequence( const OrganismSequence& orig_seq, OrganismSequence &og_seq, const unsigned int& edge_pos)
{
    //scan first
    std::vector<Element> eList;
    std::vector<unsigned int>edges;
    rt_status ret = Scan(orig_seq, eList, edges);
    if(ret.status >= RT_ERROR)
    {
        std::cout<<"Error in parsing encoded sequence: remove child sequence"<<std::endl;
        return ret;
    }

    //out of range? do nothing
    Symbol symbol;
    if(edge_pos < edges.size())
    {
        std::vector<Element>::iterator start, end;
        start = eList.begin() + edges[edge_pos];
        end = eList.begin() + start->pair_pos;
        if(start > end)
            std::swap(start, end);

        symbol =  start->symbol; //save if in case an empty eList after erasing;
        eList.erase(start, end + 1);
    }

    if(!eList.empty())
        og_seq = OrganismSequence(eList);
    else
        og_seq = OrganismSequence((robot_type)symbol.type1);

    ret.status = RT_OK;
    ret.data = orig_seq.encoded_seq.size() - eList.size();
    return ret;
}


rt_status OrganismSequence::removeChildSequence( const OrganismSequence& orig_seq, OrganismSequence &og_seq, const std::vector<unsigned int>& pos_array)
{
    //scan first
    std::vector<Element> eList;
    std::vector<unsigned int>edges;
    rt_status ret = Scan(orig_seq, eList, edges);
    if(ret.status >= RT_ERROR)
    {
        std::cout<<"Error in parsing encoded sequence"<<std::endl;
        return ret;
    }

    bool  *del = new bool[eList.size()];
    memset(del, 0, eList.size());

    //mark those need to be delete
    for (unsigned int i=0; i < pos_array.size(); i++)
    {
        if(pos_array[i] < eList.size())
        {
            unsigned int start = pos_array[i];
            unsigned int end = eList[pos_array[i]].pair_pos;
            if(start > end)
                std::swap(start, end);

            for(unsigned int j=start;j<=end;j++)
                del[j]=true;
        }
    }

    //clean the buffer
    if(!og_seq.encoded_seq.empty())
        og_seq.encoded_seq.clear();
    //copy those left
    std::vector<Element> s;
    for (unsigned int i=0; i < eList.size(); i++)
    {
        if (!del[i])
        {
            og_seq.encoded_seq.push_back(eList[i].symbol);
        }
    }

    delete []del;

    ret.status = RT_OK;
    ret.data = orig_seq.encoded_seq.size() - og_seq.encoded_seq.size(); 
    return ret;
}

rt_status OrganismSequence::extractChildSequence( const OrganismSequence& orig_seq, OrganismSequence &og_seq, const unsigned int& edge_pos, bool inclusive)
{
    //scan first
    std::vector<Element> eList;
    std::vector<unsigned int>edges;
    rt_status ret = Scan(orig_seq, eList, edges);
    if(ret.status >= RT_ERROR)
    {
        std::cout<<"Error in parsing encoded sequence : extract ChildSequence"<<std::endl;
        return ret;
    }

    if(edges.size() ==0)
    {
        ret.status = RT_OK_EXTRACT_SINGLENODE;
        og_seq = orig_seq;
        return ret;
    }

    if(edge_pos >= edges.size())
    {
        std::cout<<"position out of range: max value is "<<edges.size()-1<<std::endl;
        ret.status = RT_ERROR_POS_OUTOFRANGE;
        ret.data = 0;
        return ret;
    }
    unsigned int pos = edges[edge_pos];

    std::vector<Element> e;
    std::vector<Element>::iterator start, end;
    start = eList.begin() + pos;
    end = eList.begin() + start->pair_pos;

    if(start > end)
        std::swap(start, end);

    e.assign(start, end+1);

    if(!inclusive)
    {
        e.erase(e.begin());
        e.erase(e.end());
    }

    if(!e.empty())
        og_seq = OrganismSequence(e);
    else
        og_seq = OrganismSequence((robot_type)start->symbol.type2);

    ret.status = RT_OK;
    ret.data = og_seq.encoded_seq.size();
    return ret;
}

//extract parent sequence by the edge connecting node1 and node2, including node1's parent edge, if there is
rt_status OrganismSequence::extractParentSequence(const OrganismSequence& orig_seq, OrganismSequence &og_seq, const unsigned int& edge_pos)
{
    //scan first
    std::vector<Element> eList;
    std::vector<unsigned int>edges;
    rt_status ret = Scan(orig_seq, eList, edges);
    if(ret.status >= RT_ERROR)
    {
        std::cout<<"Error in parsing encoded sequence: extract ParentSequence"<<std::endl;
        return ret;
    }
    
    if(edges.size() ==0)
    {
        ret.status = RT_OK_EXTRACT_SINGLENODE;
        og_seq = orig_seq;
        return ret;
    }

    if(edge_pos >= edges.size())
    {
        std::cout<<"position out of range: max value is "<<edges.size()-1<<std::endl;
        ret.status = RT_ERROR_POS_OUTOFRANGE;
        ret.data = 0;
        return ret;
    }
    unsigned int pos = edges[edge_pos];

    //finding parent position
    std::vector<Element>::iterator start, end;
    start = eList.begin() + pos;
    end = eList.begin() + start->pair_pos;
    //swap if necessary
    if(start > end)
        std::swap(start, end);

    //searching backward
    bool found = false;
    while(start != eList.begin())
    {
        if((start - 1)->symbol == Symbol(0))
            start = eList.begin() + (start - 1)->pair_pos;
        else
        {
            found = true;
            start = start - 1;
            break;
        }
    }

    std::vector<Element> e;

    ret.status = RT_OK;
    if(found)
    {
        end = eList.begin() + start->pair_pos;
        e.assign(start, end+1);
    }
    else //no grand parent, copy the whole sequence
    {
        ret.status = RT_OK_NOGRANDPARENT;
        e = eList;
    }

    og_seq = OrganismSequence(e);

    ret.data = og_seq.encoded_seq.size();
    return ret;
}

rt_status OrganismSequence::mergeSequences(const OrganismSequence& og_seq1, const OrganismSequence& og_seq2, OrganismSequence& og_seq, const unsigned int& edge_pos1, const bool& parentNode, const robot_side& side1, const robot_side& side2)
{

    rt_status ret;
    rt_status ret1 = OrganismSequence::checkNodeConnection(og_seq1, edge_pos1, parentNode);
    rt_status ret2 = OrganismSequence::checkNodeConnection(og_seq2, 0, true);
    robot_type type1;

    if(ret1.status >= RT_ERROR || ret2.status >=RT_ERROR)
        return ret1;

    //check if positions are valid
    if(((ret1.data & 0xFF) & (1<<side1)) || ((ret2.data & 0xFF) & (1<<side2)))
    {
  //      std::cout<<"connection sides have already been occupied"<<std::endl;
        ret.status = RT_ERROR_POS_INUSE;
        ret.data=0;
        return ret;
    }

    //scan og_seq1
    std::vector<Element> eList;
    std::vector<unsigned int> edges;
    ret = Scan(og_seq1, eList, edges);

    if(ret.status >= RT_ERROR)
        return ret;

    unsigned int pos1;
    if(edges.size() ==0) //single node sequence
        pos1 = 0;
    else
        pos1 = edges[edge_pos1];

    std::vector<Element>::iterator start, end;
    start = eList.begin() + pos1;
    end = eList.begin() + start->pair_pos;
    if(start > end)
        std::swap(start, end);

    //find the pos that starts to search
    if(parentNode)
    {
        //search backward until it find the first sibling
        while(start > eList.begin())
        {
            if((start-1)->symbol != Symbol(0))
                break;
            else
                start = eList.begin() + (start - 1)->pair_pos;
        }
    }
    else
        start = start + 1;

    //then search forward for the insert pos
    end = eList.begin() + start->pair_pos;
    if(start->symbol!=Symbol(0))
    {
        type1 = (robot_type)start->symbol.type1;
        while(end < eList.end() - 1)
        {
            //compare
            if(start->symbol.side1 > side2 || (end + 1)->symbol == Symbol(0))
                break;
            else
            {
                start = end + 1;
                end = eList.begin() +  start->pair_pos;
            }
        }
    }
    else
    {
        //a single node sequence?
        if(eList.begin()->symbol.type2 == ROBOT_NONE)
            type1 =(robot_type)(eList.begin() + start->pair_pos)->symbol.type1;
        else
            type1 =(robot_type)(eList.begin() + start->pair_pos)->symbol.type2;
    }


    //prepare the new sub sequence
    OrganismSequence new_branch;
    new_branch.encoded_seq.push_back(Symbol(type1, side1, robot_type(og_seq2.encoded_seq[0].type1), side2));
    if(og_seq2.Edges() >0) //do nothing for single node
    {
        for(unsigned int i=0;i<og_seq2.encoded_seq.size();i++)
            new_branch.encoded_seq.push_back(og_seq2.encoded_seq[i]);
    }
    new_branch.encoded_seq.push_back(Symbol(0));

    //single node sequence
    if(eList.begin()->symbol.type2 == ROBOT_NONE)
    {
        og_seq = new_branch;
    }
    else
    {
        eList.insert(start, new_branch.encoded_seq.begin(), new_branch.encoded_seq.end());
        og_seq = OrganismSequence(eList);
    }

    ret.status = RT_OK;
    ret.data = og_seq.encoded_seq.size();
    return ret;
}

rt_status OrganismSequence::splitSequence(const OrganismSequence& og_seq, const unsigned int& edge_pos, OrganismSequence & og_seq1, OrganismSequence & og_seq2)
{
    //scan first
    std::vector<Element> eList;
    std::vector<unsigned int> edges;
    rt_status ret = Scan(og_seq, eList, edges);
    if(ret.status >= RT_ERROR)
    {
        std::cout<<"Error in parsing encoded sequence: splitSequence"<<std::endl;
        return ret;
    }

    if(edges.size() == 0)
    {
        ret.status = RT_ERROR_SPLIT_SINGLENODE;
        return ret;
    }

    if(edge_pos >= edges.size())
    {
        std::cout<<"position out of range: max value is "<<edges.size()<<std::endl;
        ret.status = RT_ERROR_POS_OUTOFRANGE;
        ret.data = 0;
        return ret;
    }

    unsigned int pos = edges[edge_pos];

 //   std::cout<<"splitting: "<<og_seq<<std::endl;
    std::vector<Element> e;
    std::vector<Element>::iterator start, end;
    start = eList.begin() + pos;
    end = eList.begin() + start->pair_pos;

 //   std::cout<<"pos: "<<pos<<" pair_pos: "<<start->pair_pos<<std::endl;

    if(start > end)
        std::swap(start, end);
    
    e.assign(start + 1, end);

    if(!e.empty())
    {
        ret = parentEdge(eList, edges, edge_pos);
        ret.edge_pos1 = ret.data;
        og_seq2 = OrganismSequence(e);
    }
    else
    {
        ret.status = RT_OK_SPLIT_NOCHILDREN;
        ret = parentEdge(eList, edges, edge_pos); // find parent edges
        ret.edge_pos1 = ret.data;
        og_seq2 = OrganismSequence((robot_type)start->symbol.type2);
    }

    ret.side1 = start->symbol.side1;
    ret.side2 = start->symbol.side2;
    ret.edge_pos2 = 0;
    Symbol symbol = start->symbol; //save it in case of empty eList after erasing
    eList.erase(start, end + 1);
    if(!eList.empty())
        og_seq1 = OrganismSequence(eList);
    else
        og_seq1 = OrganismSequence((robot_type)symbol.type1);
 //   std::cout<<"og_seq1 "<<og_seq1<<std::endl;
 //   std::cout<<"og_seq2 "<<og_seq2<<std::endl;
 //   std::cout<<"splitting done"<<std::endl;

    return ret;
}

int IRandom( int min, int max)
{
    if(max-min < 0)
        return 0;
    static bool initialised = false;
    if(!initialised)
    {
        initialised = true;
        srand(time(NULL));
    }
    return rand()%(max-min+1) + min;
}

rt_status OrganismSequence::changeNodeType(OrganismSequence& og_seq, const robot_type& type, const unsigned int& edge_pos, const bool& parentNode)
{
    std::vector<Element> eList;
    std::vector<unsigned int> edges;
    rt_status ret = Scan(og_seq, eList, edges);
    if(ret.status >= RT_ERROR)
    {
        std::cout<<"Error in parsing encoded sequence: change NodeType"<<std::endl;
        return ret;
    }

    //single node sequence
    if(ret.data == 0)
    {
        og_seq.encoded_seq[0].type1 = type;
        ret.status = RT_OK;
        return ret;
    }

    if(edge_pos >= edges.size())
    {
        std::cout<<"position out of range: max value is "<<edges.size()<<std::endl;
        ret.status = RT_ERROR_POS_OUTOFRANGE;
        ret.data = 0;
        return ret;
    }

   // std::cout<<"change node type on: "<<og_seq<<std::endl;

    unsigned int pos1 = edges[edge_pos];

    std::vector<Element>::iterator start, end;
    start = eList.begin() + pos1;
    end = eList.begin() + start->pair_pos;
    if(start > end)
        std::swap(start, end);

    //find the pos that starts to search
    bool found = false;
    if(parentNode)
    {
        found = false;
        //search backward until it find the first sibling
        while(start > eList.begin())
        {
            if((start-1)->symbol != Symbol(0))
            {
                found = true;
                start = start - 1;
                end = eList.begin() + start->pair_pos;
                break;
            }

            start = eList.begin() + (start - 1)->pair_pos;
        }

    }
    else
    {
        found = true;
        end = eList.begin() + start->pair_pos;
    }

    if(found)//there is a parent edge
    {
        //change its parent edge
        start->symbol.type2 = type;
        //change its child edges
        start = start + 1;
        while(start < end)
        {
            if(start->symbol == Symbol(0)) //shouldn't be here
                break;
            start->symbol.type1 = type;
            start = eList.begin() +  start->pair_pos + 1;
        }
    }
    else
    {
        start = eList.begin();
        end = eList.begin() + eList.size() - 1;
        while(start < end)
        {
            if(start->symbol == Symbol(0)) //shouldn't be here
                break;
            start->symbol.type1 = type;
            start = eList.begin() +  start->pair_pos + 1;
        }

    }

    og_seq = OrganismSequence(eList);
   // std::cout<<"after change: "<<og_seq<<std::endl;

    ret.status = RT_OK;

    return ret;
}

rt_status OrganismSequence::addNode(OrganismSequence& og_seq, const unsigned int& edge_pos, const bool& parentNode, const robot_type& type, const robot_side& parentSide, const robot_side& childSide)
{
    OrganismSequence single_node_seq(type);
    OrganismSequence og_seq_new;
    rt_status ret = mergeSequences(og_seq, single_node_seq, og_seq_new, edge_pos, parentNode, parentSide, childSide);
    
    if( ret.status < RT_ERROR)
    {
        og_seq = og_seq_new;
        ret.status = RT_OK;
    }

    return ret;
}
rt_status OrganismSequence::changeConnection(OrganismSequence& og_seq, const unsigned int& edge_pos)
{
    OrganismSequence sub_seq1, sub_seq2, og_seq_new;
    rt_status ret = splitSequence(og_seq, edge_pos, sub_seq1, sub_seq2);
    if(ret.status < RT_ERROR)
    {
        unsigned int edge_pos_new = IRandom(0, sub_seq1.Edges() - 1);
        bool parentNode = false;       //TODO: random 
        robot_side parentSide = (robot_side) IRandom(0, 3);
        ret = mergeSequences(sub_seq1, sub_seq2, og_seq_new, edge_pos_new, parentNode, parentSide,(robot_side)ret.side2);
        if(ret.status < RT_ERROR)
            og_seq = og_seq_new;
    }

    return ret;
}
rt_status OrganismSequence::removeSequence(OrganismSequence& og_seq, const unsigned int& edge_pos, const bool &parentNode)
{
    rt_status ret;
    OrganismSequence og_seq_new;
    if(parentNode)
        ret = removeChildSequence(og_seq, og_seq_new, edge_pos);
    else
        ret = extractChildSequence(og_seq, og_seq_new, edge_pos, false);

    if(ret.status < RT_ERROR)
    {
        og_seq = og_seq_new;
        ret.status = RT_OK;
    }

    return ret;
}


rt_status OrganismSequence::CrossOver(const OrganismSequence& parent1, const OrganismSequence& parent2, OrganismSequence& offspring1, OrganismSequence& offspring2)
{
    rt_status ret;
    if(SingleNodeSequence(parent1) || SingleNodeSequence(parent2))
    {
        ret.status = RT_ERROR_CROSSOVER_SINGLENODE;
        return ret;
    }

    unsigned int num_edges1 = parent1.encoded_seq.size() / 2;
    unsigned int num_edges2 = parent2.encoded_seq.size() / 2;

    unsigned int edge_pos1 = IRandom(0, num_edges1-1);
    unsigned int edge_pos2 = IRandom(0, num_edges2-1);

    std::cout<<"CrossOver edge pos:( "<<edge_pos1<<"\t"<<edge_pos2<<" )"<<std::endl;

    OrganismSequence sub_seq1a;
    OrganismSequence sub_seq1b;
    OrganismSequence sub_seq2a;
    OrganismSequence sub_seq2b;

    rt_status ret1 = splitSequence(parent1, edge_pos1, sub_seq1a, sub_seq1b);
    rt_status ret2 = splitSequence(parent2, edge_pos2, sub_seq2a, sub_seq2b);

    if(ret1.status >= RT_ERROR || ret2.status >= RT_ERROR)
    {
        ret.status = RT_ERROR_CROSSOVER_SPLITTING;
        return ret;
    }

    rt_status ret_merge1 = mergeSequences(sub_seq1a, sub_seq2b, offspring1,  ret1.edge_pos1, ret1.status == RT_OK_NOPARENTEDGE,  (robot_side)ret1.side1, (robot_side)ret2.side2);
    rt_status ret_merge2 = mergeSequences(sub_seq2a, sub_seq1b, offspring2,  ret2.edge_pos1, ret2.status == RT_OK_NOPARENTEDGE,  (robot_side)ret2.side1, (robot_side)ret1.side2);

    if(ret_merge1.status >= RT_ERROR || ret_merge2.status >= RT_ERROR)
    {
        ret.status = RT_ERROR_CROSSOVER_MERGING;
        return ret;
    }

    std::cout<<"parent 1: "<<parent1<<std::endl;
    std::cout<<"parent 2: "<<parent2<<std::endl;
    std::cout<<"offspring 1: "<<offspring1<<std::endl;
    std::cout<<"offspring 2: "<<offspring2<<std::endl;

    ret.status = RT_OK;
    return ret;
}

rt_status OrganismSequence::SingleNodeMutation(const OrganismSequence& og_seq, OrganismSequence& og_seq_new)
{
    og_seq_new = og_seq;
    rt_status ret;

    //first, find a leaf node, including the root node
    std::vector<Element> eList;
    std::vector<unsigned int> edges;
    ret = Scan(og_seq, eList, edges);
    if(ret.status >= RT_ERROR)
        return ret;

    std::vector<unsigned int> tmpList;
    for(unsigned int i=0;i<eList.size();i++)
    {
        if(eList[i].symbol != Symbol(0) && ( eList[i].pair_pos == i + 1 || ( i==0 && eList[i].pair_pos == eList.size()-1)))
            tmpList.push_back(i);
    }

    if(tmpList.empty())
    {
        ret.status = RT_ERROR;
        return ret;
    }

    unsigned int edge_index = IRandom(0, tmpList.size() - 1);
    unsigned int selection = IRandom(2, 4); 
    switch (selection)
    {
        case 1:
            {
                robot_type type = (robot_type) IRandom(1,2);
                //          std::cout<<"Mutation 1, change node type to "<<robottype_names[type]<<" at edge pos "<< edge_pos << (parentNode? "'s parent node" : "'s child node")<<std::endl;
                ret = changeNodeType(og_seq_new, type, tmpList[edge_index], tmpList[edge_index] == 0 ? true : false);
                break;
            }
        case 2:
            {
                //        std::cout<<"Mutation 2, remove a sequence at edge pos "<< edge_pos<<std::endl;
                ret = removeSequence(og_seq_new, tmpList[edge_index], tmpList[edge_index]==0 ? true : false);
                break;
            }
        case 3:
            {
                int edge_pos = tmpList[edge_index];//IRandom(0, edges.size() - 1);
                bool parentNode = false;//IRandom(0,1);
                robot_type type = (robot_type) IRandom(1,2);
                robot_side parentSide = (robot_side) IRandom(0, 3);
                robot_side childSide = (robot_side) IRandom(0, 3);
                //      std::cout<<"Mutation 3, add a new node (type "<<robottype_names[type]<<") at edge pos "<< edge_pos<<std::endl;
                ret = addNode(og_seq_new, edge_pos, parentNode, type, parentSide, childSide);
                break;
            }
        case 4:
            {
                //     std::cout<<"Mutation 4, change a connection at edge pos "<< edge_pos<<std::endl;
                ret = changeConnection(og_seq_new, tmpList[edge_index]);
                break;
            }
        case 5:
            {
                //       std::cout<<"Mutation 5, rotate a node at edge pos "<< edge_pos<<std::endl;
                break;
            }
        default:
            break;
    }

    return ret;
}

rt_status OrganismSequence::Mutation(const OrganismSequence& og_seq, OrganismSequence& og_seq_new)
{
    og_seq_new = og_seq;
    unsigned int edge_pos = IRandom(0, og_seq.encoded_seq.size()/2 - 1);
    unsigned int selection = IRandom(2, 4); //TODO: based on probability?

    rt_status ret;
    switch (selection)
    {
        case 1:
            {
                //bool parentNode = false;     //TODO: random
                robot_type type = (robot_type) IRandom(1,2);
      //          std::cout<<"Mutation 1, change node type to "<<robottype_names[type]<<" at edge pos "<< edge_pos << (parentNode? "'s parent node" : "'s child node")<<std::endl;
                ret = changeNodeType(og_seq_new, type, edge_pos, false);
                break;
            }
        case 2:
            {
                bool parent = false;
        //        std::cout<<"Mutation 2, remove a sequence at edge pos "<< edge_pos<<std::endl;
                ret = removeSequence(og_seq_new, edge_pos, parent);
                break;
            }
        case 3:
            {
                bool parentNode = false;     //TODO: random
                robot_type type = (robot_type) IRandom(1,2);
                robot_side parentSide = (robot_side) IRandom(0, 3);
                robot_side childSide = (robot_side) IRandom(0, 3);
          //      std::cout<<"Mutation 3, add a new node (type "<<robottype_names[type]<<") at edge pos "<< edge_pos<<std::endl;
                ret = addNode(og_seq_new, edge_pos, parentNode, type, parentSide, childSide);
                break;
            }
        case 4:
            {
           //     std::cout<<"Mutation 4, change a connection at edge pos "<< edge_pos<<std::endl;
                ret = changeConnection(og_seq_new, edge_pos);
                break;
            }
        case 5:
            {
         //       std::cout<<"Mutation 5, rotate a node at edge pos "<< edge_pos<<std::endl;
                break;
            }
        default:
            break;
    }

    return ret;
}

rt_status OrganismSequence::RandomInit(OrganismSequence& og_seq, const unsigned int size)
{
    //int size = IRandom(5, 20);
    robot_type type = (robot_type) IRandom(ROBOT_KIT, ROBOT_SCOUT);
    robot_side parentSide = (robot_side) IRandom(FRONT, LEFT);
    robot_side childSide = (robot_side) IRandom(FRONT, LEFT);
    
    //insert the first ever node
    og_seq = OrganismSequence(type);
    rt_status ret;
    OrganismSequence og_seq_new;

    for(unsigned int i=1; i<size; i++)
    {
        do
        {
            og_seq_new = OrganismSequence();
            type = ROBOT_KIT;//(robot_type) IRandom(ROBOT_KIT, ROBOT_SCOUT);
            parentSide = (robot_side) IRandom(FRONT, LEFT);
            childSide = (robot_side) IRandom(FRONT, RIGHT); //only docks to organism from FRONT and RIGHT side
            unsigned int edge_pos = IRandom(0, og_seq.Edges()-1);
          //  std::cout<<"og_seq: "<<og_seq<<std::endl;
          //  std::cout<<"add new node (type "<<robottype_names[type]<<") at edge pos "<<edge_pos<<" parentSide:"
          //      <<side_names[parentSide]<<" childSide:"<<side_names[childSide]<<std::endl;

            ret = mergeSequences(og_seq, OrganismSequence(type), og_seq_new, edge_pos, false, parentSide, childSide );
        }
        while(ret.status >= RT_ERROR);

        if(ret.status < RT_ERROR)
            og_seq = og_seq_new;
    }

    return ret;

}


