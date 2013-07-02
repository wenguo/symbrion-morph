#ifndef GASEQUENCEGENOME_HH
#define GASEQUENCEGENOME_HH

#include <ga/gaid.h>
#include <ga/GAGenome.h>
#include "organism.hh"

class GASequenceGenome : public OrganismSequence, public GAGenome {
public:
  GADeclareIdentity();

  static int Mutator(GAGenome&, float);
  static int CrossOver(const GAGenome&, const GAGenome&, GAGenome*, GAGenome*);
  static float Comparator(const GAGenome&, const GAGenome&);
  static float Evaluator(GAGenome&);
  static void RandomInit(GAGenome&);

public:
  GASequenceGenome(GAGenome::Evaluator f=NULL, void * u=NULL);
  GASequenceGenome(const GASequenceGenome&);
  GASequenceGenome & operator=(const GAGenome & orig)
    {copy(orig); return *this;}
  virtual ~GASequenceGenome();
  virtual GAGenome *clone(GAGenome::CloneMethod flag=CONTENTS) const;
  virtual void copy(const GAGenome &);
  virtual int equal(const GAGenome & c) const;

#ifdef GALIB_USE_STREAMS
  virtual int write(std::ostream&) const;
#endif
  friend std::ostream& operator<<(std::ostream&, const GASequenceGenome&);
};
#endif

