#include <ga/garandom.h>
#include <string.h>
#include "GASequenceGenome.hh"

const char * GASequenceGenome::className() const {return "GASequenceGenome";}
int GASequenceGenome::classID() const {return 999;}


GASequenceGenome::GASequenceGenome(GAGenome::Evaluator f, void * u):
    OrganismSequence(),
    GAGenome()
{
    initializer(RandomInit);
    mutator(Mutator);
    comparator(Comparator);
    evaluator(Evaluator);
    userData(u);
    crossover(CrossOver);
}

GASequenceGenome::GASequenceGenome(const GASequenceGenome& orig):
    OrganismSequence(),
    GAGenome()
{
    GASequenceGenome::copy(orig);
}
GASequenceGenome::~GASequenceGenome()
{
}
GAGenome * GASequenceGenome::clone(GAGenome::CloneMethod flag) const
{
    GASequenceGenome *cpy = new GASequenceGenome();
    if(flag ==(int)CONTENTS)
        cpy->copy(*this);
    else
        cpy->GAGenome::copy(*this);
    return cpy;
}
void GASequenceGenome::copy(const GAGenome &orig)
{
    if(&orig == this)
        return;
    const GASequenceGenome* c = DYN_CAST(const GASequenceGenome*, &orig);
    if(c)
    {
        GAGenome::copy(*c);
        OrganismSequence::copy(*c);
    }
}

#ifdef GALIB_USE_STREAMS
int GASequenceGenome::write(std::ostream& os) const
{
    os<<*this;
    return 1;
}
#endif

std::ostream& operator<<(std::ostream& os, const GASequenceGenome& seq)
{
  if(!seq.encoded_seq.empty())
    {
        for(int i=0;i<seq.encoded_seq.size();i++)
            os<<seq.encoded_seq[i];
    }
    return os;

}

int GASequenceGenome::equal(const GAGenome & c) const
{
    if(this == &c) return 1;

    const GASequenceGenome& b = DYN_CAST(const GASequenceGenome&, c);

    if(encoded_seq.size() != b.encoded_seq.size())
        return 0;

    for(unsigned int i=0;i<encoded_seq.size();i++)
    {
        if(encoded_seq[i] != b.encoded_seq[i])
            return 0;
    }

    return 1;

}

float GASequenceGenome::Evaluator(GAGenome& orig)
{
    std::cout<<"Evaluating"<<std::endl;
    //convert to graph
    GASequenceGenome &seq_genome=DYN_CAST(GASequenceGenome&, orig);
    Organism og;
    Organism::SequenceToGraph(seq_genome, og);
    og.Scan();
    if(!og.Valid() || og.NodeList().size() == 0)
        return 0;

    //find the central node
    int size = og.NodeList().size();
    int target_size = 7;

    if(size > target_size)
        return 0;

    int count = 0;
    for(int i=0;i<size;i++)
    {
        OrganismNode *node = og.NodeList()[i];
        //1. check connection on mirrored side
        uint8_t offset[2];
        memset(offset, 0, 2);
        for(int j=0;j<SIDE_COUNT;j++)
        {
            if(node->GetOrder(j) ==0)
                offset[j%2] +=1;
        }

        //if offset is odd number, count+1
        count += offset[0] % 2;
        count += offset[1] % 2;

        //2. check those with 4-2 connections, note that offset[x] should be always greater than 0
        if(offset[0] == 2 && offset[1] == 2)
        {
            if(node->GetOrder(FRONT) == node->GetOrder(BACK) ||
                    node->GetOrder(LEFT) == node->GetOrder(RIGHT))
                count -=3;
            else
                count +=1;
        }
        else if(offset[0] == 2 && offset[1] == 1)
        {
            if(node->GetOrder(FRONT) == node->GetOrder(BACK))
                count -=2;
            else
                count += abs(node->GetOrder(FRONT)-node->GetOrder(BACK)) / 2;

        }
        else if(offset[1] == 2 && offset[0] == 1)
        {
            if(node->GetOrder(LEFT) == node->GetOrder(RIGHT))
                count -=2;
            else
                count += abs(node->GetOrder(LEFT)-node->GetOrder(RIGHT)) / 2;
        }
        else if(offset[0] ==1 && offset[1] == 1)
            count+=1;

    }

    return 2*target_size - abs(target_size - size) - count;

#if 0
    OrganismNode * node=NULL;
    for(int i = 0; i < size; i++)
    {
        int count = 0;
        for(int j=0;j<SIDE_COUNT;j++)
        {
            if(og.NodeList()[i]->GetOrder(j) <= size /2)
                count++;
        }

        if(count == SIDE_COUNT)
        {
            node = og.NodeList()[i];
            break;
        }
    }

    if(node)
    {
        int offset1 = abs(node->GetOrder(FRONT) - node->GetOrder(BACK));
        int offset2 = abs(node->GetOrder(LEFT) - node->GetOrder(RIGHT));
        int offset3 = abs(node->GetDepth(FRONT) - node->GetDepth(BACK));
        int offset4 = abs(node->GetDepth(LEFT) - node->GetDepth(RIGHT));
        int sum = 2*(target_size - abs(size - target_size)) - offset1 - offset2 - offset3 - offset4;
        return sum;

    }
#endif
    return 0;

}

int GASequenceGenome::Mutator(GAGenome& orig, float pmut)
{
    if( pmut < 0.0 || !GAFlipCoin(pmut)) 
        return 0;

    std::cout<<" Mutation"<<std::endl;
    bool done = false;
    GASequenceGenome tmp;
    GASequenceGenome &seq_genome=DYN_CAST(GASequenceGenome&, orig);
    do
    {
        rt_status ret = OrganismSequence::SingleNodeMutation(seq_genome, tmp);

    // this may cause trouble as RT_ERROR_EMPTY_SEQUENCE removed 07/02/2012
    //    if(ret.status > RT_ERROR_EMPTY_SEQUENCE)
    //        continue;
    //    else 
        if(ret.status >=RT_ERROR)
            return 0;
    
        Organism og;
        Organism::SequenceToGraph(tmp, og);
        if(og.Valid())
            done = true;
    }
    while(!done);

    seq_genome = tmp;
    seq_genome._evaluated = gaFalse;
    return 1;
}

int GASequenceGenome::CrossOver(const GAGenome& p1, const GAGenome& p2, GAGenome* c1, GAGenome* c2 )
{
    std::cout<<"Crossover"<<std::endl;
    const GASequenceGenome& mom = DYN_CAST(const GASequenceGenome&, p1);
    const GASequenceGenome& dad = DYN_CAST(const GASequenceGenome&, p2);
    
    bool done = false;
    do{
        unsigned int a = GARandomInt(0, mom.Edges()-1);
        unsigned int b = GARandomInt(0, dad.Edges()-1);

        rt_status ret;
        if(c1 && c2)
        {
            OrganismSequence sub_seq1a;
            OrganismSequence sub_seq1b;
            OrganismSequence sub_seq2a;
            OrganismSequence sub_seq2b;

            GASequenceGenome &sis=DYN_CAST(GASequenceGenome&, *c1);
            GASequenceGenome &bro=DYN_CAST(GASequenceGenome&, *c2);

            sis._evaluated = gaFalse;
            bro._evaluated = gaFalse;


            rt_status ret1 = splitSequence(mom, a, sub_seq1a, sub_seq1b);
            rt_status ret2 = splitSequence(dad, b, sub_seq2a, sub_seq2b);

            if(ret1.status >= RT_ERROR || ret2.status >= RT_ERROR)
            {
                ret.status = RT_ERROR_CROSSOVER_SPLITTING;
                std::cout<<"CrossOver splitting error"<<std::endl;
                return 0;
            }

            rt_status ret_merge1 = mergeSequences(sub_seq1a, sub_seq2b, sis,  ret1.edge_pos1, ret1.status == RT_OK_NOPARENTEDGE,  (robot_side)ret1.side1, (robot_side)ret2.side2);
            rt_status ret_merge2 = mergeSequences(sub_seq2a, sub_seq1b, bro,  ret2.edge_pos1, ret2.status == RT_OK_NOPARENTEDGE,  (robot_side)ret2.side1, (robot_side)ret1.side2);

            if(ret_merge1.status >= RT_ERROR || ret_merge2.status >= RT_ERROR)
            {
                ret.status = RT_ERROR_CROSSOVER_MERGING;
                std::cout<<"CrossOver merging error"<<std::endl;
                continue;
                //return 0;
            }

            Organism og1, og2;
            Organism::SequenceToGraph(sis, og1);
            Organism::SequenceToGraph(bro, og2);
            if(og1.Valid() && og2.Valid())
                done = true;

        }
        else
            return 0;
    }
    while(!done);

    return 2;
}

float GASequenceGenome::Comparator(const GAGenome& a, const GAGenome& b)
{
    return STA_CAST(float, a.equal(b));
}

void GASequenceGenome::RandomInit(GAGenome& p1)
{
    std::cout<<"Init"<<std::endl;
    unsigned int a = GARandomInt(2, 4);
    bool done = false;
    do
    {
        GASequenceGenome& og_seq = DYN_CAST(GASequenceGenome&, p1);
        rt_status ret;
        do
        {
            ret = OrganismSequence::RandomInit(og_seq, a);
        }
        while( ret.status > RT_ERROR);

        //to validation
        Organism og;
        Organism::SequenceToGraph(og_seq, og);
        if(og.Valid())
            done = true;
    }
    while(!done);

}
