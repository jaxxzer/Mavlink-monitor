#ifndef RANGEFINDER_H
#define RANGEFINDER_H



class Rangefinder {
  public:

    Rangefinder();
  
    void init(void);
    void update(void);
  

  private:
    void range_receive(void);
  
    float range;
};

#endif
