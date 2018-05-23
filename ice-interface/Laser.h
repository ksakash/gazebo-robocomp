// **********************************************************************
//
// Copyright (c) 2003-2017 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************
//
// Ice version 3.6.4
//
// <auto-generated>
//
// Generated from file `Laser.ice'
//
// Warning: do not edit this file.
//
// </auto-generated>
//

#ifndef __Laser_h__
#define __Laser_h__

#include <IceUtil/PushDisableWarnings.h>
#include <Ice/ProxyF.h>
#include <Ice/ObjectF.h>
#include <Ice/Exception.h>
#include <Ice/LocalObject.h>
#include <Ice/StreamHelpers.h>
#include <Ice/Proxy.h>
#include <Ice/GCObject.h>
#include <Ice/AsyncResult.h>
#include <Ice/Incoming.h>
#include <IceUtil/ScopedArray.h>
#include <IceUtil/Optional.h>
#include <Ice/StreamF.h>
#include <IceUtil/UndefSysMacros.h>

#ifndef ICE_IGNORE_VERSION
#   if ICE_INT_VERSION / 100 != 306
#       error Ice version mismatch!
#   endif
#   if ICE_INT_VERSION % 100 > 50
#       error Beta header file detected
#   endif
#   if ICE_INT_VERSION % 100 < 4
#       error Ice patch level mismatch!
#   endif
#endif

namespace IceProxy
{

namespace RoboCompLaser
{

class Laser;
void __read(::IceInternal::BasicStream*, ::IceInternal::ProxyHandle< ::IceProxy::RoboCompLaser::Laser>&);
::IceProxy::Ice::Object* upCast(::IceProxy::RoboCompLaser::Laser*);

}

}

namespace RoboCompLaser
{

class Laser;
::Ice::Object* upCast(::RoboCompLaser::Laser*);
typedef ::IceInternal::Handle< ::RoboCompLaser::Laser> LaserPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::RoboCompLaser::Laser> LaserPrx;
void __patch(LaserPtr&, const ::Ice::ObjectPtr&);

}

namespace RoboCompLaser
{

typedef ::std::vector< ::Ice::Int> shortVector;

struct LaserConfData
{
    ::Ice::Int staticConf;
    ::Ice::Int maxMeasures;
    ::Ice::Int maxDegrees;
    ::Ice::Int maxRange;
    ::Ice::Int minRange;
    ::Ice::Int iniRange;
    ::Ice::Int endRange;
    ::Ice::Int cluster;
    ::Ice::Int sampleRate;
    ::Ice::Float angleRes;
    ::Ice::Float angleIni;
    ::std::string driver;
    ::std::string device;

    bool operator==(const LaserConfData& __rhs) const
    {
        if(this == &__rhs)
        {
            return true;
        }
        if(staticConf != __rhs.staticConf)
        {
            return false;
        }
        if(maxMeasures != __rhs.maxMeasures)
        {
            return false;
        }
        if(maxDegrees != __rhs.maxDegrees)
        {
            return false;
        }
        if(maxRange != __rhs.maxRange)
        {
            return false;
        }
        if(minRange != __rhs.minRange)
        {
            return false;
        }
        if(iniRange != __rhs.iniRange)
        {
            return false;
        }
        if(endRange != __rhs.endRange)
        {
            return false;
        }
        if(cluster != __rhs.cluster)
        {
            return false;
        }
        if(sampleRate != __rhs.sampleRate)
        {
            return false;
        }
        if(angleRes != __rhs.angleRes)
        {
            return false;
        }
        if(angleIni != __rhs.angleIni)
        {
            return false;
        }
        if(driver != __rhs.driver)
        {
            return false;
        }
        if(device != __rhs.device)
        {
            return false;
        }
        return true;
    }

    bool operator<(const LaserConfData& __rhs) const
    {
        if(this == &__rhs)
        {
            return false;
        }
        if(staticConf < __rhs.staticConf)
        {
            return true;
        }
        else if(__rhs.staticConf < staticConf)
        {
            return false;
        }
        if(maxMeasures < __rhs.maxMeasures)
        {
            return true;
        }
        else if(__rhs.maxMeasures < maxMeasures)
        {
            return false;
        }
        if(maxDegrees < __rhs.maxDegrees)
        {
            return true;
        }
        else if(__rhs.maxDegrees < maxDegrees)
        {
            return false;
        }
        if(maxRange < __rhs.maxRange)
        {
            return true;
        }
        else if(__rhs.maxRange < maxRange)
        {
            return false;
        }
        if(minRange < __rhs.minRange)
        {
            return true;
        }
        else if(__rhs.minRange < minRange)
        {
            return false;
        }
        if(iniRange < __rhs.iniRange)
        {
            return true;
        }
        else if(__rhs.iniRange < iniRange)
        {
            return false;
        }
        if(endRange < __rhs.endRange)
        {
            return true;
        }
        else if(__rhs.endRange < endRange)
        {
            return false;
        }
        if(cluster < __rhs.cluster)
        {
            return true;
        }
        else if(__rhs.cluster < cluster)
        {
            return false;
        }
        if(sampleRate < __rhs.sampleRate)
        {
            return true;
        }
        else if(__rhs.sampleRate < sampleRate)
        {
            return false;
        }
        if(angleRes < __rhs.angleRes)
        {
            return true;
        }
        else if(__rhs.angleRes < angleRes)
        {
            return false;
        }
        if(angleIni < __rhs.angleIni)
        {
            return true;
        }
        else if(__rhs.angleIni < angleIni)
        {
            return false;
        }
        if(driver < __rhs.driver)
        {
            return true;
        }
        else if(__rhs.driver < driver)
        {
            return false;
        }
        if(device < __rhs.device)
        {
            return true;
        }
        else if(__rhs.device < device)
        {
            return false;
        }
        return false;
    }

    bool operator!=(const LaserConfData& __rhs) const
    {
        return !operator==(__rhs);
    }
    bool operator<=(const LaserConfData& __rhs) const
    {
        return operator<(__rhs) || operator==(__rhs);
    }
    bool operator>(const LaserConfData& __rhs) const
    {
        return !operator<(__rhs) && !operator==(__rhs);
    }
    bool operator>=(const LaserConfData& __rhs) const
    {
        return !operator<(__rhs);
    }
};

struct TData
{
    ::Ice::Float angle;
    ::Ice::Float dist;

    bool operator==(const TData& __rhs) const
    {
        if(this == &__rhs)
        {
            return true;
        }
        if(angle != __rhs.angle)
        {
            return false;
        }
        if(dist != __rhs.dist)
        {
            return false;
        }
        return true;
    }

    bool operator<(const TData& __rhs) const
    {
        if(this == &__rhs)
        {
            return false;
        }
        if(angle < __rhs.angle)
        {
            return true;
        }
        else if(__rhs.angle < angle)
        {
            return false;
        }
        if(dist < __rhs.dist)
        {
            return true;
        }
        else if(__rhs.dist < dist)
        {
            return false;
        }
        return false;
    }

    bool operator!=(const TData& __rhs) const
    {
        return !operator==(__rhs);
    }
    bool operator<=(const TData& __rhs) const
    {
        return operator<(__rhs) || operator==(__rhs);
    }
    bool operator>(const TData& __rhs) const
    {
        return !operator<(__rhs) && !operator==(__rhs);
    }
    bool operator>=(const TData& __rhs) const
    {
        return !operator<(__rhs);
    }
};

typedef ::std::vector< ::RoboCompLaser::TData> TLaserData;

}

namespace Ice
{
template<>
struct StreamableTraits< ::RoboCompLaser::LaserConfData>
{
    static const StreamHelperCategory helper = StreamHelperCategoryStruct;
    static const int minWireSize = 46;
    static const bool fixedLength = false;
};

template<class S>
struct StreamWriter< ::RoboCompLaser::LaserConfData, S>
{
    static void write(S* __os, const ::RoboCompLaser::LaserConfData& v)
    {
        __os->write(v.staticConf);
        __os->write(v.maxMeasures);
        __os->write(v.maxDegrees);
        __os->write(v.maxRange);
        __os->write(v.minRange);
        __os->write(v.iniRange);
        __os->write(v.endRange);
        __os->write(v.cluster);
        __os->write(v.sampleRate);
        __os->write(v.angleRes);
        __os->write(v.angleIni);
        __os->write(v.driver);
        __os->write(v.device);
    }
};

template<class S>
struct StreamReader< ::RoboCompLaser::LaserConfData, S>
{
    static void read(S* __is, ::RoboCompLaser::LaserConfData& v)
    {
        __is->read(v.staticConf);
        __is->read(v.maxMeasures);
        __is->read(v.maxDegrees);
        __is->read(v.maxRange);
        __is->read(v.minRange);
        __is->read(v.iniRange);
        __is->read(v.endRange);
        __is->read(v.cluster);
        __is->read(v.sampleRate);
        __is->read(v.angleRes);
        __is->read(v.angleIni);
        __is->read(v.driver);
        __is->read(v.device);
    }
};

template<>
struct StreamableTraits< ::RoboCompLaser::TData>
{
    static const StreamHelperCategory helper = StreamHelperCategoryStruct;
    static const int minWireSize = 8;
    static const bool fixedLength = true;
};

template<class S>
struct StreamWriter< ::RoboCompLaser::TData, S>
{
    static void write(S* __os, const ::RoboCompLaser::TData& v)
    {
        __os->write(v.angle);
        __os->write(v.dist);
    }
};

template<class S>
struct StreamReader< ::RoboCompLaser::TData, S>
{
    static void read(S* __is, ::RoboCompLaser::TData& v)
    {
        __is->read(v.angle);
        __is->read(v.dist);
    }
};

}

namespace RoboCompLaser
{

class Callback_Laser_getLaserData_Base : virtual public ::IceInternal::CallbackBase { };
typedef ::IceUtil::Handle< Callback_Laser_getLaserData_Base> Callback_Laser_getLaserDataPtr;

class Callback_Laser_getLaserConfData_Base : virtual public ::IceInternal::CallbackBase { };
typedef ::IceUtil::Handle< Callback_Laser_getLaserConfData_Base> Callback_Laser_getLaserConfDataPtr;

}

namespace IceProxy
{

namespace RoboCompLaser
{

class Laser : virtual public ::IceProxy::Ice::Object
{
public:

    ::RoboCompLaser::TLaserData getLaserData()
    {
        return getLaserData(0);
    }
    ::RoboCompLaser::TLaserData getLaserData(const ::Ice::Context& __ctx)
    {
        return getLaserData(&__ctx);
    }
#ifdef ICE_CPP11
    ::Ice::AsyncResultPtr
    begin_getLaserData(const ::IceInternal::Function<void (const ::RoboCompLaser::TLaserData&)>& __response, const ::IceInternal::Function<void (const ::Ice::Exception&)>& __exception = ::IceInternal::Function<void (const ::Ice::Exception&)>(), const ::IceInternal::Function<void (bool)>& __sent = ::IceInternal::Function<void (bool)>())
    {
        return __begin_getLaserData(0, __response, __exception, __sent);
    }
    ::Ice::AsyncResultPtr
    begin_getLaserData(const ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>& __completed, const ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>& __sent = ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>())
    {
        return begin_getLaserData(0, ::Ice::newCallback(__completed, __sent), 0);
    }
    ::Ice::AsyncResultPtr
    begin_getLaserData(const ::Ice::Context& __ctx, const ::IceInternal::Function<void (const ::RoboCompLaser::TLaserData&)>& __response, const ::IceInternal::Function<void (const ::Ice::Exception&)>& __exception = ::IceInternal::Function<void (const ::Ice::Exception&)>(), const ::IceInternal::Function<void (bool)>& __sent = ::IceInternal::Function<void (bool)>())
    {
        return __begin_getLaserData(&__ctx, __response, __exception, __sent);
    }
    ::Ice::AsyncResultPtr
    begin_getLaserData(const ::Ice::Context& __ctx, const ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>& __completed, const ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>& __sent = ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>())
    {
        return begin_getLaserData(&__ctx, ::Ice::newCallback(__completed, __sent));
    }
    
private:

    ::Ice::AsyncResultPtr __begin_getLaserData(const ::Ice::Context* __ctx, const ::IceInternal::Function<void (const ::RoboCompLaser::TLaserData&)>& __response, const ::IceInternal::Function<void (const ::Ice::Exception&)>& __exception, const ::IceInternal::Function<void (bool)>& __sent);
    
public:
#endif

    ::Ice::AsyncResultPtr begin_getLaserData()
    {
        return begin_getLaserData(0, ::IceInternal::__dummyCallback, 0);
    }

    ::Ice::AsyncResultPtr begin_getLaserData(const ::Ice::Context& __ctx)
    {
        return begin_getLaserData(&__ctx, ::IceInternal::__dummyCallback, 0);
    }

    ::Ice::AsyncResultPtr begin_getLaserData(const ::Ice::CallbackPtr& __del, const ::Ice::LocalObjectPtr& __cookie = 0)
    {
        return begin_getLaserData(0, __del, __cookie);
    }

    ::Ice::AsyncResultPtr begin_getLaserData(const ::Ice::Context& __ctx, const ::Ice::CallbackPtr& __del, const ::Ice::LocalObjectPtr& __cookie = 0)
    {
        return begin_getLaserData(&__ctx, __del, __cookie);
    }

    ::Ice::AsyncResultPtr begin_getLaserData(const ::RoboCompLaser::Callback_Laser_getLaserDataPtr& __del, const ::Ice::LocalObjectPtr& __cookie = 0)
    {
        return begin_getLaserData(0, __del, __cookie);
    }

    ::Ice::AsyncResultPtr begin_getLaserData(const ::Ice::Context& __ctx, const ::RoboCompLaser::Callback_Laser_getLaserDataPtr& __del, const ::Ice::LocalObjectPtr& __cookie = 0)
    {
        return begin_getLaserData(&__ctx, __del, __cookie);
    }

    ::RoboCompLaser::TLaserData end_getLaserData(const ::Ice::AsyncResultPtr&);
    
private:

    ::RoboCompLaser::TLaserData getLaserData(const ::Ice::Context*);
    ::Ice::AsyncResultPtr begin_getLaserData(const ::Ice::Context*, const ::IceInternal::CallbackBasePtr&, const ::Ice::LocalObjectPtr& __cookie = 0);
    
public:

    ::RoboCompLaser::LaserConfData getLaserConfData()
    {
        return getLaserConfData(0);
    }
    ::RoboCompLaser::LaserConfData getLaserConfData(const ::Ice::Context& __ctx)
    {
        return getLaserConfData(&__ctx);
    }
#ifdef ICE_CPP11
    ::Ice::AsyncResultPtr
    begin_getLaserConfData(const ::IceInternal::Function<void (const ::RoboCompLaser::LaserConfData&)>& __response, const ::IceInternal::Function<void (const ::Ice::Exception&)>& __exception = ::IceInternal::Function<void (const ::Ice::Exception&)>(), const ::IceInternal::Function<void (bool)>& __sent = ::IceInternal::Function<void (bool)>())
    {
        return __begin_getLaserConfData(0, __response, __exception, __sent);
    }
    ::Ice::AsyncResultPtr
    begin_getLaserConfData(const ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>& __completed, const ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>& __sent = ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>())
    {
        return begin_getLaserConfData(0, ::Ice::newCallback(__completed, __sent), 0);
    }
    ::Ice::AsyncResultPtr
    begin_getLaserConfData(const ::Ice::Context& __ctx, const ::IceInternal::Function<void (const ::RoboCompLaser::LaserConfData&)>& __response, const ::IceInternal::Function<void (const ::Ice::Exception&)>& __exception = ::IceInternal::Function<void (const ::Ice::Exception&)>(), const ::IceInternal::Function<void (bool)>& __sent = ::IceInternal::Function<void (bool)>())
    {
        return __begin_getLaserConfData(&__ctx, __response, __exception, __sent);
    }
    ::Ice::AsyncResultPtr
    begin_getLaserConfData(const ::Ice::Context& __ctx, const ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>& __completed, const ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>& __sent = ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>())
    {
        return begin_getLaserConfData(&__ctx, ::Ice::newCallback(__completed, __sent));
    }
    
private:

    ::Ice::AsyncResultPtr __begin_getLaserConfData(const ::Ice::Context* __ctx, const ::IceInternal::Function<void (const ::RoboCompLaser::LaserConfData&)>& __response, const ::IceInternal::Function<void (const ::Ice::Exception&)>& __exception, const ::IceInternal::Function<void (bool)>& __sent);
    
public:
#endif

    ::Ice::AsyncResultPtr begin_getLaserConfData()
    {
        return begin_getLaserConfData(0, ::IceInternal::__dummyCallback, 0);
    }

    ::Ice::AsyncResultPtr begin_getLaserConfData(const ::Ice::Context& __ctx)
    {
        return begin_getLaserConfData(&__ctx, ::IceInternal::__dummyCallback, 0);
    }

    ::Ice::AsyncResultPtr begin_getLaserConfData(const ::Ice::CallbackPtr& __del, const ::Ice::LocalObjectPtr& __cookie = 0)
    {
        return begin_getLaserConfData(0, __del, __cookie);
    }

    ::Ice::AsyncResultPtr begin_getLaserConfData(const ::Ice::Context& __ctx, const ::Ice::CallbackPtr& __del, const ::Ice::LocalObjectPtr& __cookie = 0)
    {
        return begin_getLaserConfData(&__ctx, __del, __cookie);
    }

    ::Ice::AsyncResultPtr begin_getLaserConfData(const ::RoboCompLaser::Callback_Laser_getLaserConfDataPtr& __del, const ::Ice::LocalObjectPtr& __cookie = 0)
    {
        return begin_getLaserConfData(0, __del, __cookie);
    }

    ::Ice::AsyncResultPtr begin_getLaserConfData(const ::Ice::Context& __ctx, const ::RoboCompLaser::Callback_Laser_getLaserConfDataPtr& __del, const ::Ice::LocalObjectPtr& __cookie = 0)
    {
        return begin_getLaserConfData(&__ctx, __del, __cookie);
    }

    ::RoboCompLaser::LaserConfData end_getLaserConfData(const ::Ice::AsyncResultPtr&);
    
private:

    ::RoboCompLaser::LaserConfData getLaserConfData(const ::Ice::Context*);
    ::Ice::AsyncResultPtr begin_getLaserConfData(const ::Ice::Context*, const ::IceInternal::CallbackBasePtr&, const ::Ice::LocalObjectPtr& __cookie = 0);
    
public:
    
    ::IceInternal::ProxyHandle<Laser> ice_context(const ::Ice::Context& __context) const
    {
        return dynamic_cast<Laser*>(::IceProxy::Ice::Object::ice_context(__context).get());
    }
    
    ::IceInternal::ProxyHandle<Laser> ice_adapterId(const ::std::string& __id) const
    {
        return dynamic_cast<Laser*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    }
    
    ::IceInternal::ProxyHandle<Laser> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
        return dynamic_cast<Laser*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    }
    
    ::IceInternal::ProxyHandle<Laser> ice_locatorCacheTimeout(int __timeout) const
    {
        return dynamic_cast<Laser*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    }
    
    ::IceInternal::ProxyHandle<Laser> ice_connectionCached(bool __cached) const
    {
        return dynamic_cast<Laser*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    }
    
    ::IceInternal::ProxyHandle<Laser> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
        return dynamic_cast<Laser*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    }
    
    ::IceInternal::ProxyHandle<Laser> ice_secure(bool __secure) const
    {
        return dynamic_cast<Laser*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    }
    
    ::IceInternal::ProxyHandle<Laser> ice_preferSecure(bool __preferSecure) const
    {
        return dynamic_cast<Laser*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    }
    
    ::IceInternal::ProxyHandle<Laser> ice_router(const ::Ice::RouterPrx& __router) const
    {
        return dynamic_cast<Laser*>(::IceProxy::Ice::Object::ice_router(__router).get());
    }
    
    ::IceInternal::ProxyHandle<Laser> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
        return dynamic_cast<Laser*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    }
    
    ::IceInternal::ProxyHandle<Laser> ice_collocationOptimized(bool __co) const
    {
        return dynamic_cast<Laser*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    }
    
    ::IceInternal::ProxyHandle<Laser> ice_invocationTimeout(int __timeout) const
    {
        return dynamic_cast<Laser*>(::IceProxy::Ice::Object::ice_invocationTimeout(__timeout).get());
    }
    
    ::IceInternal::ProxyHandle<Laser> ice_twoway() const
    {
        return dynamic_cast<Laser*>(::IceProxy::Ice::Object::ice_twoway().get());
    }
    
    ::IceInternal::ProxyHandle<Laser> ice_oneway() const
    {
        return dynamic_cast<Laser*>(::IceProxy::Ice::Object::ice_oneway().get());
    }
    
    ::IceInternal::ProxyHandle<Laser> ice_batchOneway() const
    {
        return dynamic_cast<Laser*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    }
    
    ::IceInternal::ProxyHandle<Laser> ice_datagram() const
    {
        return dynamic_cast<Laser*>(::IceProxy::Ice::Object::ice_datagram().get());
    }
    
    ::IceInternal::ProxyHandle<Laser> ice_batchDatagram() const
    {
        return dynamic_cast<Laser*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    }
    
    ::IceInternal::ProxyHandle<Laser> ice_compress(bool __compress) const
    {
        return dynamic_cast<Laser*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    }
    
    ::IceInternal::ProxyHandle<Laser> ice_timeout(int __timeout) const
    {
        return dynamic_cast<Laser*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    }
    
    ::IceInternal::ProxyHandle<Laser> ice_connectionId(const ::std::string& __id) const
    {
        return dynamic_cast<Laser*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    }
    
    ::IceInternal::ProxyHandle<Laser> ice_encodingVersion(const ::Ice::EncodingVersion& __v) const
    {
        return dynamic_cast<Laser*>(::IceProxy::Ice::Object::ice_encodingVersion(__v).get());
    }
    
    static const ::std::string& ice_staticId();

private: 
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

}

}

namespace RoboCompLaser
{

class Laser : virtual public ::Ice::Object
{
public:

    typedef LaserPrx ProxyType;
    typedef LaserPtr PointerType;

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();

    virtual ::RoboCompLaser::TLaserData getLaserData(const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___getLaserData(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::RoboCompLaser::LaserConfData getLaserConfData(const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___getLaserConfData(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::Ice::DispatchStatus __dispatch(::IceInternal::Incoming&, const ::Ice::Current&);

protected:
    virtual void __writeImpl(::IceInternal::BasicStream*) const;
    virtual void __readImpl(::IceInternal::BasicStream*);
    using ::Ice::Object::__writeImpl;
    using ::Ice::Object::__readImpl;
};

inline bool operator==(const Laser& l, const Laser& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

inline bool operator<(const Laser& l, const Laser& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

}

namespace RoboCompLaser
{

template<class T>
class CallbackNC_Laser_getLaserData : public Callback_Laser_getLaserData_Base, public ::IceInternal::TwowayCallbackNC<T>
{
public:

    typedef IceUtil::Handle<T> TPtr;

    typedef void (T::*Exception)(const ::Ice::Exception&);
    typedef void (T::*Sent)(bool);
    typedef void (T::*Response)(const ::RoboCompLaser::TLaserData&);

    CallbackNC_Laser_getLaserData(const TPtr& obj, Response cb, Exception excb, Sent sentcb)
        : ::IceInternal::TwowayCallbackNC<T>(obj, cb != 0, excb, sentcb), _response(cb)
    {
    }

    virtual void completed(const ::Ice::AsyncResultPtr& __result) const
    {
        ::RoboCompLaser::LaserPrx __proxy = ::RoboCompLaser::LaserPrx::uncheckedCast(__result->getProxy());
        ::RoboCompLaser::TLaserData __ret;
        try
        {
            __ret = __proxy->end_getLaserData(__result);
        }
        catch(const ::Ice::Exception& ex)
        {
            ::IceInternal::CallbackNC<T>::exception(__result, ex);
            return;
        }
        if(_response)
        {
            (::IceInternal::CallbackNC<T>::_callback.get()->*_response)(__ret);
        }
    }

    private:

    Response _response;
};

template<class T> Callback_Laser_getLaserDataPtr
newCallback_Laser_getLaserData(const IceUtil::Handle<T>& instance, void (T::*cb)(const ::RoboCompLaser::TLaserData&), void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_Laser_getLaserData<T>(instance, cb, excb, sentcb);
}

template<class T> Callback_Laser_getLaserDataPtr
newCallback_Laser_getLaserData(T* instance, void (T::*cb)(const ::RoboCompLaser::TLaserData&), void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_Laser_getLaserData<T>(instance, cb, excb, sentcb);
}

template<class T, typename CT>
class Callback_Laser_getLaserData : public Callback_Laser_getLaserData_Base, public ::IceInternal::TwowayCallback<T, CT>
{
public:

    typedef IceUtil::Handle<T> TPtr;

    typedef void (T::*Exception)(const ::Ice::Exception& , const CT&);
    typedef void (T::*Sent)(bool , const CT&);
    typedef void (T::*Response)(const ::RoboCompLaser::TLaserData&, const CT&);

    Callback_Laser_getLaserData(const TPtr& obj, Response cb, Exception excb, Sent sentcb)
        : ::IceInternal::TwowayCallback<T, CT>(obj, cb != 0, excb, sentcb), _response(cb)
    {
    }

    virtual void completed(const ::Ice::AsyncResultPtr& __result) const
    {
        ::RoboCompLaser::LaserPrx __proxy = ::RoboCompLaser::LaserPrx::uncheckedCast(__result->getProxy());
        ::RoboCompLaser::TLaserData __ret;
        try
        {
            __ret = __proxy->end_getLaserData(__result);
        }
        catch(const ::Ice::Exception& ex)
        {
            ::IceInternal::Callback<T, CT>::exception(__result, ex);
            return;
        }
        if(_response)
        {
            (::IceInternal::Callback<T, CT>::_callback.get()->*_response)(__ret, CT::dynamicCast(__result->getCookie()));
        }
    }

    private:

    Response _response;
};

template<class T, typename CT> Callback_Laser_getLaserDataPtr
newCallback_Laser_getLaserData(const IceUtil::Handle<T>& instance, void (T::*cb)(const ::RoboCompLaser::TLaserData&, const CT&), void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_Laser_getLaserData<T, CT>(instance, cb, excb, sentcb);
}

template<class T, typename CT> Callback_Laser_getLaserDataPtr
newCallback_Laser_getLaserData(T* instance, void (T::*cb)(const ::RoboCompLaser::TLaserData&, const CT&), void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_Laser_getLaserData<T, CT>(instance, cb, excb, sentcb);
}

template<class T>
class CallbackNC_Laser_getLaserConfData : public Callback_Laser_getLaserConfData_Base, public ::IceInternal::TwowayCallbackNC<T>
{
public:

    typedef IceUtil::Handle<T> TPtr;

    typedef void (T::*Exception)(const ::Ice::Exception&);
    typedef void (T::*Sent)(bool);
    typedef void (T::*Response)(const ::RoboCompLaser::LaserConfData&);

    CallbackNC_Laser_getLaserConfData(const TPtr& obj, Response cb, Exception excb, Sent sentcb)
        : ::IceInternal::TwowayCallbackNC<T>(obj, cb != 0, excb, sentcb), _response(cb)
    {
    }

    virtual void completed(const ::Ice::AsyncResultPtr& __result) const
    {
        ::RoboCompLaser::LaserPrx __proxy = ::RoboCompLaser::LaserPrx::uncheckedCast(__result->getProxy());
        ::RoboCompLaser::LaserConfData __ret;
        try
        {
            __ret = __proxy->end_getLaserConfData(__result);
        }
        catch(const ::Ice::Exception& ex)
        {
            ::IceInternal::CallbackNC<T>::exception(__result, ex);
            return;
        }
        if(_response)
        {
            (::IceInternal::CallbackNC<T>::_callback.get()->*_response)(__ret);
        }
    }

    private:

    Response _response;
};

template<class T> Callback_Laser_getLaserConfDataPtr
newCallback_Laser_getLaserConfData(const IceUtil::Handle<T>& instance, void (T::*cb)(const ::RoboCompLaser::LaserConfData&), void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_Laser_getLaserConfData<T>(instance, cb, excb, sentcb);
}

template<class T> Callback_Laser_getLaserConfDataPtr
newCallback_Laser_getLaserConfData(T* instance, void (T::*cb)(const ::RoboCompLaser::LaserConfData&), void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_Laser_getLaserConfData<T>(instance, cb, excb, sentcb);
}

template<class T, typename CT>
class Callback_Laser_getLaserConfData : public Callback_Laser_getLaserConfData_Base, public ::IceInternal::TwowayCallback<T, CT>
{
public:

    typedef IceUtil::Handle<T> TPtr;

    typedef void (T::*Exception)(const ::Ice::Exception& , const CT&);
    typedef void (T::*Sent)(bool , const CT&);
    typedef void (T::*Response)(const ::RoboCompLaser::LaserConfData&, const CT&);

    Callback_Laser_getLaserConfData(const TPtr& obj, Response cb, Exception excb, Sent sentcb)
        : ::IceInternal::TwowayCallback<T, CT>(obj, cb != 0, excb, sentcb), _response(cb)
    {
    }

    virtual void completed(const ::Ice::AsyncResultPtr& __result) const
    {
        ::RoboCompLaser::LaserPrx __proxy = ::RoboCompLaser::LaserPrx::uncheckedCast(__result->getProxy());
        ::RoboCompLaser::LaserConfData __ret;
        try
        {
            __ret = __proxy->end_getLaserConfData(__result);
        }
        catch(const ::Ice::Exception& ex)
        {
            ::IceInternal::Callback<T, CT>::exception(__result, ex);
            return;
        }
        if(_response)
        {
            (::IceInternal::Callback<T, CT>::_callback.get()->*_response)(__ret, CT::dynamicCast(__result->getCookie()));
        }
    }

    private:

    Response _response;
};

template<class T, typename CT> Callback_Laser_getLaserConfDataPtr
newCallback_Laser_getLaserConfData(const IceUtil::Handle<T>& instance, void (T::*cb)(const ::RoboCompLaser::LaserConfData&, const CT&), void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_Laser_getLaserConfData<T, CT>(instance, cb, excb, sentcb);
}

template<class T, typename CT> Callback_Laser_getLaserConfDataPtr
newCallback_Laser_getLaserConfData(T* instance, void (T::*cb)(const ::RoboCompLaser::LaserConfData&, const CT&), void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_Laser_getLaserConfData<T, CT>(instance, cb, excb, sentcb);
}

}

#include <IceUtil/PopDisableWarnings.h>
#endif
