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
// Generated from file `Motors.ice'
//
// Warning: do not edit this file.
//
// </auto-generated>
//

#ifndef ___home_ironman_gazebo_robocomp_slice____slice_cpp__Motors_h__
#define ___home_ironman_gazebo_robocomp_slice____slice_cpp__Motors_h__

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

namespace RoboCompMotors
{

class Motors;
void __read(::IceInternal::BasicStream*, ::IceInternal::ProxyHandle< ::IceProxy::RoboCompMotors::Motors>&);
::IceProxy::Ice::Object* upCast(::IceProxy::RoboCompMotors::Motors*);

}

}

namespace RoboCompMotors
{

class Motors;
::Ice::Object* upCast(::RoboCompMotors::Motors*);
typedef ::IceInternal::Handle< ::RoboCompMotors::Motors> MotorsPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::RoboCompMotors::Motors> MotorsPrx;
void __patch(MotorsPtr&, const ::Ice::ObjectPtr&);

}

namespace RoboCompMotors
{

class Callback_Motors_getMotorSpeed_Base : virtual public ::IceInternal::CallbackBase { };
typedef ::IceUtil::Handle< Callback_Motors_getMotorSpeed_Base> Callback_Motors_getMotorSpeedPtr;

class Callback_Motors_setMotorSpeed_Base : virtual public ::IceInternal::CallbackBase { };
typedef ::IceUtil::Handle< Callback_Motors_setMotorSpeed_Base> Callback_Motors_setMotorSpeedPtr;

}

namespace IceProxy
{

namespace RoboCompMotors
{

class Motors : virtual public ::IceProxy::Ice::Object
{
public:

    ::Ice::Float getMotorSpeed()
    {
        return getMotorSpeed(0);
    }
    ::Ice::Float getMotorSpeed(const ::Ice::Context& __ctx)
    {
        return getMotorSpeed(&__ctx);
    }
#ifdef ICE_CPP11
    ::Ice::AsyncResultPtr
    begin_getMotorSpeed(const ::IceInternal::Function<void (::Ice::Float)>& __response, const ::IceInternal::Function<void (const ::Ice::Exception&)>& __exception = ::IceInternal::Function<void (const ::Ice::Exception&)>(), const ::IceInternal::Function<void (bool)>& __sent = ::IceInternal::Function<void (bool)>())
    {
        return __begin_getMotorSpeed(0, __response, __exception, __sent);
    }
    ::Ice::AsyncResultPtr
    begin_getMotorSpeed(const ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>& __completed, const ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>& __sent = ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>())
    {
        return begin_getMotorSpeed(0, ::Ice::newCallback(__completed, __sent), 0);
    }
    ::Ice::AsyncResultPtr
    begin_getMotorSpeed(const ::Ice::Context& __ctx, const ::IceInternal::Function<void (::Ice::Float)>& __response, const ::IceInternal::Function<void (const ::Ice::Exception&)>& __exception = ::IceInternal::Function<void (const ::Ice::Exception&)>(), const ::IceInternal::Function<void (bool)>& __sent = ::IceInternal::Function<void (bool)>())
    {
        return __begin_getMotorSpeed(&__ctx, __response, __exception, __sent);
    }
    ::Ice::AsyncResultPtr
    begin_getMotorSpeed(const ::Ice::Context& __ctx, const ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>& __completed, const ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>& __sent = ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>())
    {
        return begin_getMotorSpeed(&__ctx, ::Ice::newCallback(__completed, __sent));
    }
    
private:

    ::Ice::AsyncResultPtr __begin_getMotorSpeed(const ::Ice::Context* __ctx, const ::IceInternal::Function<void (::Ice::Float)>& __response, const ::IceInternal::Function<void (const ::Ice::Exception&)>& __exception, const ::IceInternal::Function<void (bool)>& __sent);
    
public:
#endif

    ::Ice::AsyncResultPtr begin_getMotorSpeed()
    {
        return begin_getMotorSpeed(0, ::IceInternal::__dummyCallback, 0);
    }

    ::Ice::AsyncResultPtr begin_getMotorSpeed(const ::Ice::Context& __ctx)
    {
        return begin_getMotorSpeed(&__ctx, ::IceInternal::__dummyCallback, 0);
    }

    ::Ice::AsyncResultPtr begin_getMotorSpeed(const ::Ice::CallbackPtr& __del, const ::Ice::LocalObjectPtr& __cookie = 0)
    {
        return begin_getMotorSpeed(0, __del, __cookie);
    }

    ::Ice::AsyncResultPtr begin_getMotorSpeed(const ::Ice::Context& __ctx, const ::Ice::CallbackPtr& __del, const ::Ice::LocalObjectPtr& __cookie = 0)
    {
        return begin_getMotorSpeed(&__ctx, __del, __cookie);
    }

    ::Ice::AsyncResultPtr begin_getMotorSpeed(const ::RoboCompMotors::Callback_Motors_getMotorSpeedPtr& __del, const ::Ice::LocalObjectPtr& __cookie = 0)
    {
        return begin_getMotorSpeed(0, __del, __cookie);
    }

    ::Ice::AsyncResultPtr begin_getMotorSpeed(const ::Ice::Context& __ctx, const ::RoboCompMotors::Callback_Motors_getMotorSpeedPtr& __del, const ::Ice::LocalObjectPtr& __cookie = 0)
    {
        return begin_getMotorSpeed(&__ctx, __del, __cookie);
    }

    ::Ice::Float end_getMotorSpeed(const ::Ice::AsyncResultPtr&);
    
private:

    ::Ice::Float getMotorSpeed(const ::Ice::Context*);
    ::Ice::AsyncResultPtr begin_getMotorSpeed(const ::Ice::Context*, const ::IceInternal::CallbackBasePtr&, const ::Ice::LocalObjectPtr& __cookie = 0);
    
public:

    void setMotorSpeed(::Ice::Float __p_w)
    {
        setMotorSpeed(__p_w, 0);
    }
    void setMotorSpeed(::Ice::Float __p_w, const ::Ice::Context& __ctx)
    {
        setMotorSpeed(__p_w, &__ctx);
    }
#ifdef ICE_CPP11
    ::Ice::AsyncResultPtr
    begin_setMotorSpeed(::Ice::Float __p_w, const ::IceInternal::Function<void ()>& __response, const ::IceInternal::Function<void (const ::Ice::Exception&)>& __exception = ::IceInternal::Function<void (const ::Ice::Exception&)>(), const ::IceInternal::Function<void (bool)>& __sent = ::IceInternal::Function<void (bool)>())
    {
        return begin_setMotorSpeed(__p_w, 0, new ::IceInternal::Cpp11FnOnewayCallbackNC(__response, __exception, __sent));
    }
    ::Ice::AsyncResultPtr
    begin_setMotorSpeed(::Ice::Float __p_w, const ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>& __completed, const ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>& __sent = ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>())
    {
        return begin_setMotorSpeed(__p_w, 0, ::Ice::newCallback(__completed, __sent), 0);
    }
    ::Ice::AsyncResultPtr
    begin_setMotorSpeed(::Ice::Float __p_w, const ::Ice::Context& __ctx, const ::IceInternal::Function<void ()>& __response, const ::IceInternal::Function<void (const ::Ice::Exception&)>& __exception = ::IceInternal::Function<void (const ::Ice::Exception&)>(), const ::IceInternal::Function<void (bool)>& __sent = ::IceInternal::Function<void (bool)>())
    {
        return begin_setMotorSpeed(__p_w, &__ctx, new ::IceInternal::Cpp11FnOnewayCallbackNC(__response, __exception, __sent), 0);
    }
    ::Ice::AsyncResultPtr
    begin_setMotorSpeed(::Ice::Float __p_w, const ::Ice::Context& __ctx, const ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>& __completed, const ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>& __sent = ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>())
    {
        return begin_setMotorSpeed(__p_w, &__ctx, ::Ice::newCallback(__completed, __sent));
    }
#endif

    ::Ice::AsyncResultPtr begin_setMotorSpeed(::Ice::Float __p_w)
    {
        return begin_setMotorSpeed(__p_w, 0, ::IceInternal::__dummyCallback, 0);
    }

    ::Ice::AsyncResultPtr begin_setMotorSpeed(::Ice::Float __p_w, const ::Ice::Context& __ctx)
    {
        return begin_setMotorSpeed(__p_w, &__ctx, ::IceInternal::__dummyCallback, 0);
    }

    ::Ice::AsyncResultPtr begin_setMotorSpeed(::Ice::Float __p_w, const ::Ice::CallbackPtr& __del, const ::Ice::LocalObjectPtr& __cookie = 0)
    {
        return begin_setMotorSpeed(__p_w, 0, __del, __cookie);
    }

    ::Ice::AsyncResultPtr begin_setMotorSpeed(::Ice::Float __p_w, const ::Ice::Context& __ctx, const ::Ice::CallbackPtr& __del, const ::Ice::LocalObjectPtr& __cookie = 0)
    {
        return begin_setMotorSpeed(__p_w, &__ctx, __del, __cookie);
    }

    ::Ice::AsyncResultPtr begin_setMotorSpeed(::Ice::Float __p_w, const ::RoboCompMotors::Callback_Motors_setMotorSpeedPtr& __del, const ::Ice::LocalObjectPtr& __cookie = 0)
    {
        return begin_setMotorSpeed(__p_w, 0, __del, __cookie);
    }

    ::Ice::AsyncResultPtr begin_setMotorSpeed(::Ice::Float __p_w, const ::Ice::Context& __ctx, const ::RoboCompMotors::Callback_Motors_setMotorSpeedPtr& __del, const ::Ice::LocalObjectPtr& __cookie = 0)
    {
        return begin_setMotorSpeed(__p_w, &__ctx, __del, __cookie);
    }

    void end_setMotorSpeed(const ::Ice::AsyncResultPtr&);
    
private:

    void setMotorSpeed(::Ice::Float, const ::Ice::Context*);
    ::Ice::AsyncResultPtr begin_setMotorSpeed(::Ice::Float, const ::Ice::Context*, const ::IceInternal::CallbackBasePtr&, const ::Ice::LocalObjectPtr& __cookie = 0);
    
public:
    
    ::IceInternal::ProxyHandle<Motors> ice_context(const ::Ice::Context& __context) const
    {
        return dynamic_cast<Motors*>(::IceProxy::Ice::Object::ice_context(__context).get());
    }
    
    ::IceInternal::ProxyHandle<Motors> ice_adapterId(const ::std::string& __id) const
    {
        return dynamic_cast<Motors*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    }
    
    ::IceInternal::ProxyHandle<Motors> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
        return dynamic_cast<Motors*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    }
    
    ::IceInternal::ProxyHandle<Motors> ice_locatorCacheTimeout(int __timeout) const
    {
        return dynamic_cast<Motors*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    }
    
    ::IceInternal::ProxyHandle<Motors> ice_connectionCached(bool __cached) const
    {
        return dynamic_cast<Motors*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    }
    
    ::IceInternal::ProxyHandle<Motors> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
        return dynamic_cast<Motors*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    }
    
    ::IceInternal::ProxyHandle<Motors> ice_secure(bool __secure) const
    {
        return dynamic_cast<Motors*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    }
    
    ::IceInternal::ProxyHandle<Motors> ice_preferSecure(bool __preferSecure) const
    {
        return dynamic_cast<Motors*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    }
    
    ::IceInternal::ProxyHandle<Motors> ice_router(const ::Ice::RouterPrx& __router) const
    {
        return dynamic_cast<Motors*>(::IceProxy::Ice::Object::ice_router(__router).get());
    }
    
    ::IceInternal::ProxyHandle<Motors> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
        return dynamic_cast<Motors*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    }
    
    ::IceInternal::ProxyHandle<Motors> ice_collocationOptimized(bool __co) const
    {
        return dynamic_cast<Motors*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    }
    
    ::IceInternal::ProxyHandle<Motors> ice_invocationTimeout(int __timeout) const
    {
        return dynamic_cast<Motors*>(::IceProxy::Ice::Object::ice_invocationTimeout(__timeout).get());
    }
    
    ::IceInternal::ProxyHandle<Motors> ice_twoway() const
    {
        return dynamic_cast<Motors*>(::IceProxy::Ice::Object::ice_twoway().get());
    }
    
    ::IceInternal::ProxyHandle<Motors> ice_oneway() const
    {
        return dynamic_cast<Motors*>(::IceProxy::Ice::Object::ice_oneway().get());
    }
    
    ::IceInternal::ProxyHandle<Motors> ice_batchOneway() const
    {
        return dynamic_cast<Motors*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    }
    
    ::IceInternal::ProxyHandle<Motors> ice_datagram() const
    {
        return dynamic_cast<Motors*>(::IceProxy::Ice::Object::ice_datagram().get());
    }
    
    ::IceInternal::ProxyHandle<Motors> ice_batchDatagram() const
    {
        return dynamic_cast<Motors*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    }
    
    ::IceInternal::ProxyHandle<Motors> ice_compress(bool __compress) const
    {
        return dynamic_cast<Motors*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    }
    
    ::IceInternal::ProxyHandle<Motors> ice_timeout(int __timeout) const
    {
        return dynamic_cast<Motors*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    }
    
    ::IceInternal::ProxyHandle<Motors> ice_connectionId(const ::std::string& __id) const
    {
        return dynamic_cast<Motors*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    }
    
    ::IceInternal::ProxyHandle<Motors> ice_encodingVersion(const ::Ice::EncodingVersion& __v) const
    {
        return dynamic_cast<Motors*>(::IceProxy::Ice::Object::ice_encodingVersion(__v).get());
    }
    
    static const ::std::string& ice_staticId();

private: 
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

}

}

namespace RoboCompMotors
{

class Motors : virtual public ::Ice::Object
{
public:

    typedef MotorsPrx ProxyType;
    typedef MotorsPtr PointerType;

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();

    virtual ::Ice::Float getMotorSpeed(const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___getMotorSpeed(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void setMotorSpeed(::Ice::Float, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___setMotorSpeed(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::Ice::DispatchStatus __dispatch(::IceInternal::Incoming&, const ::Ice::Current&);

protected:
    virtual void __writeImpl(::IceInternal::BasicStream*) const;
    virtual void __readImpl(::IceInternal::BasicStream*);
    using ::Ice::Object::__writeImpl;
    using ::Ice::Object::__readImpl;
};

inline bool operator==(const Motors& l, const Motors& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

inline bool operator<(const Motors& l, const Motors& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

}

namespace RoboCompMotors
{

template<class T>
class CallbackNC_Motors_getMotorSpeed : public Callback_Motors_getMotorSpeed_Base, public ::IceInternal::TwowayCallbackNC<T>
{
public:

    typedef IceUtil::Handle<T> TPtr;

    typedef void (T::*Exception)(const ::Ice::Exception&);
    typedef void (T::*Sent)(bool);
    typedef void (T::*Response)(::Ice::Float);

    CallbackNC_Motors_getMotorSpeed(const TPtr& obj, Response cb, Exception excb, Sent sentcb)
        : ::IceInternal::TwowayCallbackNC<T>(obj, cb != 0, excb, sentcb), _response(cb)
    {
    }

    virtual void completed(const ::Ice::AsyncResultPtr& __result) const
    {
        ::RoboCompMotors::MotorsPrx __proxy = ::RoboCompMotors::MotorsPrx::uncheckedCast(__result->getProxy());
        ::Ice::Float __ret;
        try
        {
            __ret = __proxy->end_getMotorSpeed(__result);
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

template<class T> Callback_Motors_getMotorSpeedPtr
newCallback_Motors_getMotorSpeed(const IceUtil::Handle<T>& instance, void (T::*cb)(::Ice::Float), void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_Motors_getMotorSpeed<T>(instance, cb, excb, sentcb);
}

template<class T> Callback_Motors_getMotorSpeedPtr
newCallback_Motors_getMotorSpeed(T* instance, void (T::*cb)(::Ice::Float), void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_Motors_getMotorSpeed<T>(instance, cb, excb, sentcb);
}

template<class T, typename CT>
class Callback_Motors_getMotorSpeed : public Callback_Motors_getMotorSpeed_Base, public ::IceInternal::TwowayCallback<T, CT>
{
public:

    typedef IceUtil::Handle<T> TPtr;

    typedef void (T::*Exception)(const ::Ice::Exception& , const CT&);
    typedef void (T::*Sent)(bool , const CT&);
    typedef void (T::*Response)(::Ice::Float, const CT&);

    Callback_Motors_getMotorSpeed(const TPtr& obj, Response cb, Exception excb, Sent sentcb)
        : ::IceInternal::TwowayCallback<T, CT>(obj, cb != 0, excb, sentcb), _response(cb)
    {
    }

    virtual void completed(const ::Ice::AsyncResultPtr& __result) const
    {
        ::RoboCompMotors::MotorsPrx __proxy = ::RoboCompMotors::MotorsPrx::uncheckedCast(__result->getProxy());
        ::Ice::Float __ret;
        try
        {
            __ret = __proxy->end_getMotorSpeed(__result);
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

template<class T, typename CT> Callback_Motors_getMotorSpeedPtr
newCallback_Motors_getMotorSpeed(const IceUtil::Handle<T>& instance, void (T::*cb)(::Ice::Float, const CT&), void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_Motors_getMotorSpeed<T, CT>(instance, cb, excb, sentcb);
}

template<class T, typename CT> Callback_Motors_getMotorSpeedPtr
newCallback_Motors_getMotorSpeed(T* instance, void (T::*cb)(::Ice::Float, const CT&), void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_Motors_getMotorSpeed<T, CT>(instance, cb, excb, sentcb);
}

template<class T>
class CallbackNC_Motors_setMotorSpeed : public Callback_Motors_setMotorSpeed_Base, public ::IceInternal::OnewayCallbackNC<T>
{
public:

    typedef IceUtil::Handle<T> TPtr;

    typedef void (T::*Exception)(const ::Ice::Exception&);
    typedef void (T::*Sent)(bool);
    typedef void (T::*Response)();

    CallbackNC_Motors_setMotorSpeed(const TPtr& obj, Response cb, Exception excb, Sent sentcb)
        : ::IceInternal::OnewayCallbackNC<T>(obj, cb, excb, sentcb)
    {
    }
};

template<class T> Callback_Motors_setMotorSpeedPtr
newCallback_Motors_setMotorSpeed(const IceUtil::Handle<T>& instance, void (T::*cb)(), void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_Motors_setMotorSpeed<T>(instance, cb, excb, sentcb);
}

template<class T> Callback_Motors_setMotorSpeedPtr
newCallback_Motors_setMotorSpeed(const IceUtil::Handle<T>& instance, void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_Motors_setMotorSpeed<T>(instance, 0, excb, sentcb);
}

template<class T> Callback_Motors_setMotorSpeedPtr
newCallback_Motors_setMotorSpeed(T* instance, void (T::*cb)(), void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_Motors_setMotorSpeed<T>(instance, cb, excb, sentcb);
}

template<class T> Callback_Motors_setMotorSpeedPtr
newCallback_Motors_setMotorSpeed(T* instance, void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_Motors_setMotorSpeed<T>(instance, 0, excb, sentcb);
}

template<class T, typename CT>
class Callback_Motors_setMotorSpeed : public Callback_Motors_setMotorSpeed_Base, public ::IceInternal::OnewayCallback<T, CT>
{
public:

    typedef IceUtil::Handle<T> TPtr;

    typedef void (T::*Exception)(const ::Ice::Exception& , const CT&);
    typedef void (T::*Sent)(bool , const CT&);
    typedef void (T::*Response)(const CT&);

    Callback_Motors_setMotorSpeed(const TPtr& obj, Response cb, Exception excb, Sent sentcb)
        : ::IceInternal::OnewayCallback<T, CT>(obj, cb, excb, sentcb)
    {
    }
};

template<class T, typename CT> Callback_Motors_setMotorSpeedPtr
newCallback_Motors_setMotorSpeed(const IceUtil::Handle<T>& instance, void (T::*cb)(const CT&), void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_Motors_setMotorSpeed<T, CT>(instance, cb, excb, sentcb);
}

template<class T, typename CT> Callback_Motors_setMotorSpeedPtr
newCallback_Motors_setMotorSpeed(const IceUtil::Handle<T>& instance, void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_Motors_setMotorSpeed<T, CT>(instance, 0, excb, sentcb);
}

template<class T, typename CT> Callback_Motors_setMotorSpeedPtr
newCallback_Motors_setMotorSpeed(T* instance, void (T::*cb)(const CT&), void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_Motors_setMotorSpeed<T, CT>(instance, cb, excb, sentcb);
}

template<class T, typename CT> Callback_Motors_setMotorSpeedPtr
newCallback_Motors_setMotorSpeed(T* instance, void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_Motors_setMotorSpeed<T, CT>(instance, 0, excb, sentcb);
}

}

#include <IceUtil/PopDisableWarnings.h>
#endif
