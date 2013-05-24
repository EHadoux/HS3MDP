#ifndef BELIEF_STATE_H
#define BELIEF_STATE_H

#include <vector>

class STATE;
class SIMULATOR;

class BELIEF_STATE
{
public:

    BELIEF_STATE();
    virtual ~BELIEF_STATE();

    // Free memory for all states
    virtual void Free(const SIMULATOR& simulator);

    // Creates new state, now owned by caller
    virtual STATE* CreateSample(const SIMULATOR& simulator) const;

    // Added state is owned by belief state
    virtual void AddSample(STATE* state);

    // Make own copies of all samples
    virtual void Copy(const BELIEF_STATE* beliefs, const SIMULATOR& simulator);

    // Move all samples into this belief state
    virtual void Move(BELIEF_STATE* beliefs);

    virtual bool Empty() const { return Samples.empty(); }
    virtual int GetNumSamples() const { return Samples.size(); }
    virtual const STATE* GetSample(int index) const { return Samples[index]; }
    
private:
    std::vector<STATE*> Samples;
};

#endif // BELIEF_STATE_H
