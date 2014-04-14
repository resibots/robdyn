

#ifndef KEYBOARD_H
#define KEYBOARD_H

#include <iostream>
#include <osgGA/GUIEventHandler>

namespace renderer
{
  class KeyboardEventHandler : public osgGA::GUIEventHandler
  {
  public:
    
    typedef void (*functionType) ();
    
    enum keyStatusType { KEY_UP, KEY_DOWN };
    
    struct functionStatusType
    {
      functionStatusType() {keyState = KEY_UP; keyFunction = NULL;}
      functionType keyFunction;
      keyStatusType keyState;
    };
    
    bool addFunction(int whatKey, functionType newFunction);  
    bool addFunction(int whatKey, keyStatusType keyPressStatus, functionType newFunction);  
    virtual bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter&);  
    virtual void accept(osgGA::GUIEventHandlerVisitor& v)   { v.visit(*this); };  
  protected:  
    typedef std::map<int, functionStatusType > keyFunctionMap;
    keyFunctionMap keyFuncMap;
    keyFunctionMap keyUPFuncMap;  
  };
}

#endif
