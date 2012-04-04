// 
//
// Author: Wenguo Liu
// Date: 11/2011
//
//
#ifndef ORGANISM_HH
#define ORGANISM_HH
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <math.h>
#include <stdint.h>

#ifdef STAGE
#define unit 0.09
#else
#define unit 100.0
#endif

enum robot_type {ROBOT_NONE = 0x0, ROBOT_KIT = 0x1, ROBOT_SCOUT = 0x2, ROBOT_AW = 0x3, ROBOTTYPE_COUNT};
enum robot_side {FRONT=0x0, RIGHT, BACK, LEFT, SIDE_COUNT};
extern char robottype_names[ROBOTTYPE_COUNT];
extern char side_names[SIDE_COUNT];

enum status_type
{
    RT_OK = 0x0,
    RT_OK_NOGRANDPARENT,
    RT_OK_EMPTY_BRANCH,
    RT_OK_EMPTY_SEQUENCE,
    RT_OK_EXTRACT_SINGLENODE,
    RT_OK_SPLIT_NOCHILDREN,
    RT_OK_NOPARENTEDGE,
    RT_ERROR = 0x0100,
    RT_ERROR_PARSE,
    RT_ERROR_POS_OUTOFRANGE,
    RT_ERROR_POS_INUSE,
    RT_ERROR_SPLIT_SINGLENODE,
    RT_ERROR_CROSSOVER_SINGLENODE,
    RT_ERROR_CROSSOVER_SPLITTING,
    RT_ERROR_CROSSOVER_MERGING
};

inline double DTOR(double x)
{
    return x*3.1415926/180.0;
};

struct rt_status
{
    status_type status;
    union{
        uint32_t  data;
        struct{
            uint32_t side1:2;
            uint32_t edge_pos1:14;
            uint32_t side2:2;
            uint32_t edge_pos2:14;
        };
    };
};

struct visited_status
{
    bool status;
    robot_side parent_side;
};

//sequence representation
class OrganismSequence
{
    friend class Organism;
    public:
        class Symbol
        {
            public:
                Symbol(uint8_t sym = 0);
                void reBuild(const char str[4]);
                Symbol(const robot_type& t1, const robot_side& s1,const robot_type& t2, const robot_side& s2);
                Symbol(const Symbol&);
                Symbol& operator=(const Symbol&);
                bool operator==(const Symbol&) const;
                bool operator!=(const Symbol&) const;

                union
                {
                    uint8_t data;
                    struct{
                        uint8_t type1:2; //bits: 0 - 1
                        uint8_t side1:2; //bits: 2 - 3
                        uint8_t type2:2; //bits: 4 - 5
                        uint8_t side2:2; //bits: 6 - 7
                    };
                };
            private:
                uint8_t getType(const char type);
                uint8_t getSide(const char side);
        };
        class Element
        {
            public:
                Element();
                Element(const Symbol& sym):symbol(sym){};
                Element(const Element&);
                Element& operator=(const Element&);
                ~Element();
                Symbol symbol;
                uint32_t pair_pos;
        };

    public:
        OrganismSequence();
        OrganismSequence(const robot_type& t); //single node sequence
        OrganismSequence(const OrganismSequence&);
        ~OrganismSequence();

        void copy(const OrganismSequence&);
        bool compare(const OrganismSequence&);

        static rt_status RandomInit(OrganismSequence&, const unsigned int size = 10);

        OrganismSequence& operator=(const OrganismSequence&);
        bool operator==(const OrganismSequence&) const;

        //genetic operator
        static rt_status CrossOver(const OrganismSequence&, const OrganismSequence&, OrganismSequence&, OrganismSequence&);
        static rt_status Mutation(const OrganismSequence&, OrganismSequence&);
        static rt_status SingleNodeMutation(const OrganismSequence&, OrganismSequence&);


        static rt_status Scan(const OrganismSequence&, std::vector<Element>&, std::vector<unsigned int>&);
        static rt_status fillBranches(const OrganismSequence&, std::vector<OrganismSequence>&);
        static rt_status removeChildSequence( const OrganismSequence&, OrganismSequence&, const unsigned int&edge_pos);
        static rt_status removeChildSequence( const OrganismSequence&, OrganismSequence&, const std::vector<unsigned int>&);
        static rt_status extractChildSequence(const OrganismSequence&, OrganismSequence&, const unsigned int&edge_pos, bool inclusive=true);
        static rt_status extractParentSequence(const OrganismSequence&, OrganismSequence&, const unsigned int&edge_pos);
        static rt_status mergeSequences(const OrganismSequence& og_seq1, const OrganismSequence& og_seq2, OrganismSequence& og_seq, const unsigned int& edge_pos1, const bool& flag, const robot_side& side1, const robot_side& side2);
        static rt_status splitSequence(const OrganismSequence& og_seq, const unsigned int& edge_pos, OrganismSequence& og_seq1, OrganismSequence& og_seq2);
        static rt_status checkNodeConnection(const OrganismSequence& og_seq, const unsigned int& edge_pos, const bool& parentNode);
        static bool SingleNodeSequence(const OrganismSequence& og_seq);

		static uint8_t maxCommonTreeSize( OrganismSequence& seq1, OrganismSequence& seq2 );

        static rt_status changeNodeType(OrganismSequence&, const robot_type&, const unsigned int& edge_pos, const bool&);
        static rt_status addNode(OrganismSequence&, const unsigned int& edge_pos, const bool& parentNode, const robot_type& type, const robot_side& parentSide, const robot_side& childSide);
        static rt_status changeConnection(OrganismSequence&, const unsigned int& edge_pos);
        static rt_status removeSequence(OrganismSequence&, const unsigned int& edge_pos, const bool&);

        // for self-repair
        static OrganismSequence getNextSeedSeq(OrganismSequence&);

        rt_status Scan(std::vector<Element>&, std::vector<unsigned int>&) const;
        rt_status getBranch(OrganismSequence&, const robot_side&);
        rt_status getBranch(OrganismSequence&, const robot_side&, bool );
        const Symbol getSymbol(const unsigned int& pos) const;
        inline unsigned int Size() const {return encoded_seq.size();}
        const unsigned int Edges() const;
        inline void Clear() {encoded_seq.clear();}
        rt_status reBuild(const uint8_t *data, const int size);
        rt_status reBuild(const char *str);
        inline const std::vector<Symbol>& Encoded_Seq() const{return encoded_seq;}

        friend std::ostream& operator<<(std::ostream&, const Symbol&);
        friend std::ostream& operator<<(std::ostream&, const Element&);
        friend std::ostream& operator<<(std::ostream&, const OrganismSequence&);

    private:
        OrganismSequence(const std::vector<Element>& eList);
        OrganismSequence(const std::vector<Symbol>& sList);
        static rt_status parentEdge(std::vector<Element>&, const std::vector<unsigned int>&, const unsigned int& edge);

    protected:
        std::vector<Symbol> encoded_seq;
};

//node class in graph
class OrganismNode
{
    friend class Organism;
    public:
        OrganismNode(const robot_type& t = ROBOT_KIT);
        OrganismNode(int index, const robot_type&);

        //for drawing an organism in simulation
        class Geom
        {
            public:
                Geom();
                Geom(const Geom &c);
                ~Geom() {};
                void Draw(bool draw_name=false);
                double Distance(const Geom&);

                static void BuildList();// generate buildlist for opengl
                static unsigned int body[ROBOTTYPE_COUNT];
                static double geom_size[ROBOTTYPE_COUNT][3];

                double px;
                double py;
                double pa;
                double width;
                double height;
                double length;

                OrganismNode * parent;//the node attached to
        };

        void GetNewGeomFromParent(const OrganismNode &parent, const robot_side& parentSide, const robot_side& mySide);

        bool FindConnectionSide(robot_side &side, const OrganismNode &node);
        void ResetStatus();

        inline Geom& GetGeom(){return geom;};
        inline unsigned int GetDepth(int index){return depth[index];}
        inline unsigned int GetOrder(int index){return order[index];}

        friend std::ostream& operator<<(std::ostream&, const Geom&);
        friend std::ostream& operator<<(std::ostream&, const OrganismNode&);


    private:
        static int id_index;
        robot_type type;
        int id;
        visited_status visited[SIDE_COUNT];
        OrganismNode* connection[SIDE_COUNT];
        unsigned int depth[SIDE_COUNT]; 
        unsigned int order[SIDE_COUNT];
        OrganismSequence * seq;  //corresponding sequence representation for graph seeding from itself
        Geom geom;
};

class Organism
{
    public:
        Organism();
        ~Organism();

        OrganismNode* Insert(OrganismNode *parent, OrganismNode *child, const robot_side& parentSide=BACK, const robot_side& childSide=FRONT);
        static void GraphToSequence(OrganismNode &node, OrganismSequence &seq);
        static rt_status SequenceToGraph(const OrganismSequence &seq, Organism & og);
        
        const std::vector<OrganismNode*>& NodeList(){return nodeList;}
        const std::vector<OrganismSequence*>& SeqList(){return seqList;}
        const char * Name(){return name;}
        void GraphToSequence(); //generate all sequences
        void GraphToSequence(OrganismSequence &seq); //generate a sequence corresponding to the first node
        void Scan();

        bool Valid();
        void Clear();

        void Draw(double orig_x=0, double orig_y=0, double z=0, double rot=0, bool draw_name = false);
        friend std::ostream& operator<<(std::ostream&, const Organism&);

        void ResetStatus();
    private:
        static void DFSTraversal(OrganismNode &node, OrganismSequence &seq);
        static void OGSeqTraversal(const OrganismSequence &og_seq, Organism& og, OrganismNode* parent = NULL);
        static void DFSTraversal(OrganismNode &node, std::vector<OrganismNode*>& orderedNodeList);

        char *name;
        std::vector<OrganismNode*> nodeList; // store all nodes in graph, though all are connected
        std::vector<OrganismSequence*> seqList; // store sequence represenation, correpsonds to the node in nodeList
};


int IRandom( int min, int max);

#endif
