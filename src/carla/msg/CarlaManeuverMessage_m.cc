//
// Generated file, do not edit! Created by opp_msgtool 6.0 from carla/msg/CarlaManeuverMessage.msg.
//

// Disable warnings about unused variables, empty switch stmts, etc:
#ifdef _MSC_VER
#  pragma warning(disable:4101)
#  pragma warning(disable:4065)
#endif

#if defined(__clang__)
#  pragma clang diagnostic ignored "-Wshadow"
#  pragma clang diagnostic ignored "-Wconversion"
#  pragma clang diagnostic ignored "-Wunused-parameter"
#  pragma clang diagnostic ignored "-Wc++98-compat"
#  pragma clang diagnostic ignored "-Wunreachable-code-break"
#  pragma clang diagnostic ignored "-Wold-style-cast"
#elif defined(__GNUC__)
#  pragma GCC diagnostic ignored "-Wshadow"
#  pragma GCC diagnostic ignored "-Wconversion"
#  pragma GCC diagnostic ignored "-Wunused-parameter"
#  pragma GCC diagnostic ignored "-Wold-style-cast"
#  pragma GCC diagnostic ignored "-Wsuggest-attribute=noreturn"
#  pragma GCC diagnostic ignored "-Wfloat-conversion"
#endif

#include <iostream>
#include <sstream>
#include <memory>
#include <type_traits>
#include "CarlaManeuverMessage_m.h"

namespace omnetpp {

// Template pack/unpack rules. They are declared *after* a1l type-specific pack functions for multiple reasons.
// They are in the omnetpp namespace, to allow them to be found by argument-dependent lookup via the cCommBuffer argument

// Packing/unpacking an std::vector
template<typename T, typename A>
void doParsimPacking(omnetpp::cCommBuffer *buffer, const std::vector<T,A>& v)
{
    int n = v.size();
    doParsimPacking(buffer, n);
    for (int i = 0; i < n; i++)
        doParsimPacking(buffer, v[i]);
}

template<typename T, typename A>
void doParsimUnpacking(omnetpp::cCommBuffer *buffer, std::vector<T,A>& v)
{
    int n;
    doParsimUnpacking(buffer, n);
    v.resize(n);
    for (int i = 0; i < n; i++)
        doParsimUnpacking(buffer, v[i]);
}

// Packing/unpacking an std::list
template<typename T, typename A>
void doParsimPacking(omnetpp::cCommBuffer *buffer, const std::list<T,A>& l)
{
    doParsimPacking(buffer, (int)l.size());
    for (typename std::list<T,A>::const_iterator it = l.begin(); it != l.end(); ++it)
        doParsimPacking(buffer, (T&)*it);
}

template<typename T, typename A>
void doParsimUnpacking(omnetpp::cCommBuffer *buffer, std::list<T,A>& l)
{
    int n;
    doParsimUnpacking(buffer, n);
    for (int i = 0; i < n; i++) {
        l.push_back(T());
        doParsimUnpacking(buffer, l.back());
    }
}

// Packing/unpacking an std::set
template<typename T, typename Tr, typename A>
void doParsimPacking(omnetpp::cCommBuffer *buffer, const std::set<T,Tr,A>& s)
{
    doParsimPacking(buffer, (int)s.size());
    for (typename std::set<T,Tr,A>::const_iterator it = s.begin(); it != s.end(); ++it)
        doParsimPacking(buffer, *it);
}

template<typename T, typename Tr, typename A>
void doParsimUnpacking(omnetpp::cCommBuffer *buffer, std::set<T,Tr,A>& s)
{
    int n;
    doParsimUnpacking(buffer, n);
    for (int i = 0; i < n; i++) {
        T x;
        doParsimUnpacking(buffer, x);
        s.insert(x);
    }
}

// Packing/unpacking an std::map
template<typename K, typename V, typename Tr, typename A>
void doParsimPacking(omnetpp::cCommBuffer *buffer, const std::map<K,V,Tr,A>& m)
{
    doParsimPacking(buffer, (int)m.size());
    for (typename std::map<K,V,Tr,A>::const_iterator it = m.begin(); it != m.end(); ++it) {
        doParsimPacking(buffer, it->first);
        doParsimPacking(buffer, it->second);
    }
}

template<typename K, typename V, typename Tr, typename A>
void doParsimUnpacking(omnetpp::cCommBuffer *buffer, std::map<K,V,Tr,A>& m)
{
    int n;
    doParsimUnpacking(buffer, n);
    for (int i = 0; i < n; i++) {
        K k; V v;
        doParsimUnpacking(buffer, k);
        doParsimUnpacking(buffer, v);
        m[k] = v;
    }
}

// Default pack/unpack function for arrays
template<typename T>
void doParsimArrayPacking(omnetpp::cCommBuffer *b, const T *t, int n)
{
    for (int i = 0; i < n; i++)
        doParsimPacking(b, t[i]);
}

template<typename T>
void doParsimArrayUnpacking(omnetpp::cCommBuffer *b, T *t, int n)
{
    for (int i = 0; i < n; i++)
        doParsimUnpacking(b, t[i]);
}

// Default rule to prevent compiler from choosing base class' doParsimPacking() function
template<typename T>
void doParsimPacking(omnetpp::cCommBuffer *, const T& t)
{
    throw omnetpp::cRuntimeError("Parsim error: No doParsimPacking() function for type %s", omnetpp::opp_typename(typeid(t)));
}

template<typename T>
void doParsimUnpacking(omnetpp::cCommBuffer *, T& t)
{
    throw omnetpp::cRuntimeError("Parsim error: No doParsimUnpacking() function for type %s", omnetpp::opp_typename(typeid(t)));
}

}  // namespace omnetpp

Register_Enum(CarlaManeuverType, (CarlaManeuverType::JOIN_REQ, CarlaManeuverType::JOIN_RSP, CarlaManeuverType::MOVE_TO_POS, CarlaManeuverType::MOVE_TO_POS_ACK, CarlaManeuverType::JOIN_FORMATION, CarlaManeuverType::JOIN_FORMATION_ACK, CarlaManeuverType::UPDATE_FORMATION));

Register_Class(CarlaManeuverMessage)

CarlaManeuverMessage::CarlaManeuverMessage(const char *name) : ::omnetpp::cPacket(name)
{
}

CarlaManeuverMessage::CarlaManeuverMessage(const CarlaManeuverMessage& other) : ::omnetpp::cPacket(other)
{
    copy(other);
}

CarlaManeuverMessage::~CarlaManeuverMessage()
{
    delete [] this->newFormation;
}

CarlaManeuverMessage& CarlaManeuverMessage::operator=(const CarlaManeuverMessage& other)
{
    if (this == &other) return *this;
    ::omnetpp::cPacket::operator=(other);
    copy(other);
    return *this;
}

void CarlaManeuverMessage::copy(const CarlaManeuverMessage& other)
{
    this->msgType = other.msgType;
    this->senderId = other.senderId;
    this->platoonId = other.platoonId;
    this->leaderId = other.leaderId;
    this->joinerId = other.joinerId;
    this->permitted = other.permitted;
    this->platoonSpeed = other.platoonSpeed;
    this->joinIndex = other.joinIndex;
    delete [] this->newFormation;
    this->newFormation = (other.newFormation_arraysize==0) ? nullptr : new int[other.newFormation_arraysize];
    newFormation_arraysize = other.newFormation_arraysize;
    for (size_t i = 0; i < newFormation_arraysize; i++) {
        this->newFormation[i] = other.newFormation[i];
    }
}

void CarlaManeuverMessage::parsimPack(omnetpp::cCommBuffer *b) const
{
    ::omnetpp::cPacket::parsimPack(b);
    doParsimPacking(b,this->msgType);
    doParsimPacking(b,this->senderId);
    doParsimPacking(b,this->platoonId);
    doParsimPacking(b,this->leaderId);
    doParsimPacking(b,this->joinerId);
    doParsimPacking(b,this->permitted);
    doParsimPacking(b,this->platoonSpeed);
    doParsimPacking(b,this->joinIndex);
    b->pack(newFormation_arraysize);
    doParsimArrayPacking(b,this->newFormation,newFormation_arraysize);
}

void CarlaManeuverMessage::parsimUnpack(omnetpp::cCommBuffer *b)
{
    ::omnetpp::cPacket::parsimUnpack(b);
    doParsimUnpacking(b,this->msgType);
    doParsimUnpacking(b,this->senderId);
    doParsimUnpacking(b,this->platoonId);
    doParsimUnpacking(b,this->leaderId);
    doParsimUnpacking(b,this->joinerId);
    doParsimUnpacking(b,this->permitted);
    doParsimUnpacking(b,this->platoonSpeed);
    doParsimUnpacking(b,this->joinIndex);
    delete [] this->newFormation;
    b->unpack(newFormation_arraysize);
    if (newFormation_arraysize == 0) {
        this->newFormation = nullptr;
    } else {
        this->newFormation = new int[newFormation_arraysize];
        doParsimArrayUnpacking(b,this->newFormation,newFormation_arraysize);
    }
}

int CarlaManeuverMessage::getMsgType() const
{
    return this->msgType;
}

void CarlaManeuverMessage::setMsgType(int msgType)
{
    this->msgType = msgType;
}

int CarlaManeuverMessage::getSenderId() const
{
    return this->senderId;
}

void CarlaManeuverMessage::setSenderId(int senderId)
{
    this->senderId = senderId;
}

int CarlaManeuverMessage::getPlatoonId() const
{
    return this->platoonId;
}

void CarlaManeuverMessage::setPlatoonId(int platoonId)
{
    this->platoonId = platoonId;
}

int CarlaManeuverMessage::getLeaderId() const
{
    return this->leaderId;
}

void CarlaManeuverMessage::setLeaderId(int leaderId)
{
    this->leaderId = leaderId;
}

int CarlaManeuverMessage::getJoinerId() const
{
    return this->joinerId;
}

void CarlaManeuverMessage::setJoinerId(int joinerId)
{
    this->joinerId = joinerId;
}

bool CarlaManeuverMessage::getPermitted() const
{
    return this->permitted;
}

void CarlaManeuverMessage::setPermitted(bool permitted)
{
    this->permitted = permitted;
}

double CarlaManeuverMessage::getPlatoonSpeed() const
{
    return this->platoonSpeed;
}

void CarlaManeuverMessage::setPlatoonSpeed(double platoonSpeed)
{
    this->platoonSpeed = platoonSpeed;
}

int CarlaManeuverMessage::getJoinIndex() const
{
    return this->joinIndex;
}

void CarlaManeuverMessage::setJoinIndex(int joinIndex)
{
    this->joinIndex = joinIndex;
}

size_t CarlaManeuverMessage::getNewFormationArraySize() const
{
    return newFormation_arraysize;
}

int CarlaManeuverMessage::getNewFormation(size_t k) const
{
    if (k >= newFormation_arraysize) throw omnetpp::cRuntimeError("Array of size %lu indexed by %lu", (unsigned long)newFormation_arraysize, (unsigned long)k);
    return this->newFormation[k];
}

void CarlaManeuverMessage::setNewFormationArraySize(size_t newSize)
{
    int *newFormation2 = (newSize==0) ? nullptr : new int[newSize];
    size_t minSize = newFormation_arraysize < newSize ? newFormation_arraysize : newSize;
    for (size_t i = 0; i < minSize; i++)
        newFormation2[i] = this->newFormation[i];
    for (size_t i = minSize; i < newSize; i++)
        newFormation2[i] = 0;
    delete [] this->newFormation;
    this->newFormation = newFormation2;
    newFormation_arraysize = newSize;
}

void CarlaManeuverMessage::setNewFormation(size_t k, int newFormation)
{
    if (k >= newFormation_arraysize) throw omnetpp::cRuntimeError("Array of size %lu indexed by %lu", (unsigned long)newFormation_arraysize, (unsigned long)k);
    this->newFormation[k] = newFormation;
}

void CarlaManeuverMessage::insertNewFormation(size_t k, int newFormation)
{
    if (k > newFormation_arraysize) throw omnetpp::cRuntimeError("Array of size %lu indexed by %lu", (unsigned long)newFormation_arraysize, (unsigned long)k);
    size_t newSize = newFormation_arraysize + 1;
    int *newFormation2 = new int[newSize];
    size_t i;
    for (i = 0; i < k; i++)
        newFormation2[i] = this->newFormation[i];
    newFormation2[k] = newFormation;
    for (i = k + 1; i < newSize; i++)
        newFormation2[i] = this->newFormation[i-1];
    delete [] this->newFormation;
    this->newFormation = newFormation2;
    newFormation_arraysize = newSize;
}

void CarlaManeuverMessage::appendNewFormation(int newFormation)
{
    insertNewFormation(newFormation_arraysize, newFormation);
}

void CarlaManeuverMessage::eraseNewFormation(size_t k)
{
    if (k >= newFormation_arraysize) throw omnetpp::cRuntimeError("Array of size %lu indexed by %lu", (unsigned long)newFormation_arraysize, (unsigned long)k);
    size_t newSize = newFormation_arraysize - 1;
    int *newFormation2 = (newSize == 0) ? nullptr : new int[newSize];
    size_t i;
    for (i = 0; i < k; i++)
        newFormation2[i] = this->newFormation[i];
    for (i = k; i < newSize; i++)
        newFormation2[i] = this->newFormation[i+1];
    delete [] this->newFormation;
    this->newFormation = newFormation2;
    newFormation_arraysize = newSize;
}

class CarlaManeuverMessageDescriptor : public omnetpp::cClassDescriptor
{
  private:
    mutable const char **propertyNames;
    enum FieldConstants {
        FIELD_msgType,
        FIELD_senderId,
        FIELD_platoonId,
        FIELD_leaderId,
        FIELD_joinerId,
        FIELD_permitted,
        FIELD_platoonSpeed,
        FIELD_joinIndex,
        FIELD_newFormation,
    };
  public:
    CarlaManeuverMessageDescriptor();
    virtual ~CarlaManeuverMessageDescriptor();

    virtual bool doesSupport(omnetpp::cObject *obj) const override;
    virtual const char **getPropertyNames() const override;
    virtual const char *getProperty(const char *propertyName) const override;
    virtual int getFieldCount() const override;
    virtual const char *getFieldName(int field) const override;
    virtual int findField(const char *fieldName) const override;
    virtual unsigned int getFieldTypeFlags(int field) const override;
    virtual const char *getFieldTypeString(int field) const override;
    virtual const char **getFieldPropertyNames(int field) const override;
    virtual const char *getFieldProperty(int field, const char *propertyName) const override;
    virtual int getFieldArraySize(omnetpp::any_ptr object, int field) const override;
    virtual void setFieldArraySize(omnetpp::any_ptr object, int field, int size) const override;

    virtual const char *getFieldDynamicTypeString(omnetpp::any_ptr object, int field, int i) const override;
    virtual std::string getFieldValueAsString(omnetpp::any_ptr object, int field, int i) const override;
    virtual void setFieldValueAsString(omnetpp::any_ptr object, int field, int i, const char *value) const override;
    virtual omnetpp::cValue getFieldValue(omnetpp::any_ptr object, int field, int i) const override;
    virtual void setFieldValue(omnetpp::any_ptr object, int field, int i, const omnetpp::cValue& value) const override;

    virtual const char *getFieldStructName(int field) const override;
    virtual omnetpp::any_ptr getFieldStructValuePointer(omnetpp::any_ptr object, int field, int i) const override;
    virtual void setFieldStructValuePointer(omnetpp::any_ptr object, int field, int i, omnetpp::any_ptr ptr) const override;
};

Register_ClassDescriptor(CarlaManeuverMessageDescriptor)

CarlaManeuverMessageDescriptor::CarlaManeuverMessageDescriptor() : omnetpp::cClassDescriptor(omnetpp::opp_typename(typeid(CarlaManeuverMessage)), "omnetpp::cPacket")
{
    propertyNames = nullptr;
}

CarlaManeuverMessageDescriptor::~CarlaManeuverMessageDescriptor()
{
    delete[] propertyNames;
}

bool CarlaManeuverMessageDescriptor::doesSupport(omnetpp::cObject *obj) const
{
    return dynamic_cast<CarlaManeuverMessage *>(obj)!=nullptr;
}

const char **CarlaManeuverMessageDescriptor::getPropertyNames() const
{
    if (!propertyNames) {
        static const char *names[] = {  nullptr };
        omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
        const char **baseNames = base ? base->getPropertyNames() : nullptr;
        propertyNames = mergeLists(baseNames, names);
    }
    return propertyNames;
}

const char *CarlaManeuverMessageDescriptor::getProperty(const char *propertyName) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    return base ? base->getProperty(propertyName) : nullptr;
}

int CarlaManeuverMessageDescriptor::getFieldCount() const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    return base ? 9+base->getFieldCount() : 9;
}

unsigned int CarlaManeuverMessageDescriptor::getFieldTypeFlags(int field) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldTypeFlags(field);
        field -= base->getFieldCount();
    }
    static unsigned int fieldTypeFlags[] = {
        FD_ISEDITABLE,    // FIELD_msgType
        FD_ISEDITABLE,    // FIELD_senderId
        FD_ISEDITABLE,    // FIELD_platoonId
        FD_ISEDITABLE,    // FIELD_leaderId
        FD_ISEDITABLE,    // FIELD_joinerId
        FD_ISEDITABLE,    // FIELD_permitted
        FD_ISEDITABLE,    // FIELD_platoonSpeed
        FD_ISEDITABLE,    // FIELD_joinIndex
        FD_ISARRAY | FD_ISEDITABLE | FD_ISRESIZABLE,    // FIELD_newFormation
    };
    return (field >= 0 && field < 9) ? fieldTypeFlags[field] : 0;
}

const char *CarlaManeuverMessageDescriptor::getFieldName(int field) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldName(field);
        field -= base->getFieldCount();
    }
    static const char *fieldNames[] = {
        "msgType",
        "senderId",
        "platoonId",
        "leaderId",
        "joinerId",
        "permitted",
        "platoonSpeed",
        "joinIndex",
        "newFormation",
    };
    return (field >= 0 && field < 9) ? fieldNames[field] : nullptr;
}

int CarlaManeuverMessageDescriptor::findField(const char *fieldName) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    int baseIndex = base ? base->getFieldCount() : 0;
    if (strcmp(fieldName, "msgType") == 0) return baseIndex + 0;
    if (strcmp(fieldName, "senderId") == 0) return baseIndex + 1;
    if (strcmp(fieldName, "platoonId") == 0) return baseIndex + 2;
    if (strcmp(fieldName, "leaderId") == 0) return baseIndex + 3;
    if (strcmp(fieldName, "joinerId") == 0) return baseIndex + 4;
    if (strcmp(fieldName, "permitted") == 0) return baseIndex + 5;
    if (strcmp(fieldName, "platoonSpeed") == 0) return baseIndex + 6;
    if (strcmp(fieldName, "joinIndex") == 0) return baseIndex + 7;
    if (strcmp(fieldName, "newFormation") == 0) return baseIndex + 8;
    return base ? base->findField(fieldName) : -1;
}

const char *CarlaManeuverMessageDescriptor::getFieldTypeString(int field) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldTypeString(field);
        field -= base->getFieldCount();
    }
    static const char *fieldTypeStrings[] = {
        "int",    // FIELD_msgType
        "int",    // FIELD_senderId
        "int",    // FIELD_platoonId
        "int",    // FIELD_leaderId
        "int",    // FIELD_joinerId
        "bool",    // FIELD_permitted
        "double",    // FIELD_platoonSpeed
        "int",    // FIELD_joinIndex
        "int",    // FIELD_newFormation
    };
    return (field >= 0 && field < 9) ? fieldTypeStrings[field] : nullptr;
}

const char **CarlaManeuverMessageDescriptor::getFieldPropertyNames(int field) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldPropertyNames(field);
        field -= base->getFieldCount();
    }
    switch (field) {
        default: return nullptr;
    }
}

const char *CarlaManeuverMessageDescriptor::getFieldProperty(int field, const char *propertyName) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldProperty(field, propertyName);
        field -= base->getFieldCount();
    }
    switch (field) {
        default: return nullptr;
    }
}

int CarlaManeuverMessageDescriptor::getFieldArraySize(omnetpp::any_ptr object, int field) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldArraySize(object, field);
        field -= base->getFieldCount();
    }
    CarlaManeuverMessage *pp = omnetpp::fromAnyPtr<CarlaManeuverMessage>(object); (void)pp;
    switch (field) {
        case FIELD_newFormation: return pp->getNewFormationArraySize();
        default: return 0;
    }
}

void CarlaManeuverMessageDescriptor::setFieldArraySize(omnetpp::any_ptr object, int field, int size) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount()){
            base->setFieldArraySize(object, field, size);
            return;
        }
        field -= base->getFieldCount();
    }
    CarlaManeuverMessage *pp = omnetpp::fromAnyPtr<CarlaManeuverMessage>(object); (void)pp;
    switch (field) {
        case FIELD_newFormation: pp->setNewFormationArraySize(size); break;
        default: throw omnetpp::cRuntimeError("Cannot set array size of field %d of class 'CarlaManeuverMessage'", field);
    }
}

const char *CarlaManeuverMessageDescriptor::getFieldDynamicTypeString(omnetpp::any_ptr object, int field, int i) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldDynamicTypeString(object,field,i);
        field -= base->getFieldCount();
    }
    CarlaManeuverMessage *pp = omnetpp::fromAnyPtr<CarlaManeuverMessage>(object); (void)pp;
    switch (field) {
        default: return nullptr;
    }
}

std::string CarlaManeuverMessageDescriptor::getFieldValueAsString(omnetpp::any_ptr object, int field, int i) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldValueAsString(object,field,i);
        field -= base->getFieldCount();
    }
    CarlaManeuverMessage *pp = omnetpp::fromAnyPtr<CarlaManeuverMessage>(object); (void)pp;
    switch (field) {
        case FIELD_msgType: return long2string(pp->getMsgType());
        case FIELD_senderId: return long2string(pp->getSenderId());
        case FIELD_platoonId: return long2string(pp->getPlatoonId());
        case FIELD_leaderId: return long2string(pp->getLeaderId());
        case FIELD_joinerId: return long2string(pp->getJoinerId());
        case FIELD_permitted: return bool2string(pp->getPermitted());
        case FIELD_platoonSpeed: return double2string(pp->getPlatoonSpeed());
        case FIELD_joinIndex: return long2string(pp->getJoinIndex());
        case FIELD_newFormation: return long2string(pp->getNewFormation(i));
        default: return "";
    }
}

void CarlaManeuverMessageDescriptor::setFieldValueAsString(omnetpp::any_ptr object, int field, int i, const char *value) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount()){
            base->setFieldValueAsString(object, field, i, value);
            return;
        }
        field -= base->getFieldCount();
    }
    CarlaManeuverMessage *pp = omnetpp::fromAnyPtr<CarlaManeuverMessage>(object); (void)pp;
    switch (field) {
        case FIELD_msgType: pp->setMsgType(string2long(value)); break;
        case FIELD_senderId: pp->setSenderId(string2long(value)); break;
        case FIELD_platoonId: pp->setPlatoonId(string2long(value)); break;
        case FIELD_leaderId: pp->setLeaderId(string2long(value)); break;
        case FIELD_joinerId: pp->setJoinerId(string2long(value)); break;
        case FIELD_permitted: pp->setPermitted(string2bool(value)); break;
        case FIELD_platoonSpeed: pp->setPlatoonSpeed(string2double(value)); break;
        case FIELD_joinIndex: pp->setJoinIndex(string2long(value)); break;
        case FIELD_newFormation: pp->setNewFormation(i,string2long(value)); break;
        default: throw omnetpp::cRuntimeError("Cannot set field %d of class 'CarlaManeuverMessage'", field);
    }
}

omnetpp::cValue CarlaManeuverMessageDescriptor::getFieldValue(omnetpp::any_ptr object, int field, int i) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldValue(object,field,i);
        field -= base->getFieldCount();
    }
    CarlaManeuverMessage *pp = omnetpp::fromAnyPtr<CarlaManeuverMessage>(object); (void)pp;
    switch (field) {
        case FIELD_msgType: return pp->getMsgType();
        case FIELD_senderId: return pp->getSenderId();
        case FIELD_platoonId: return pp->getPlatoonId();
        case FIELD_leaderId: return pp->getLeaderId();
        case FIELD_joinerId: return pp->getJoinerId();
        case FIELD_permitted: return pp->getPermitted();
        case FIELD_platoonSpeed: return pp->getPlatoonSpeed();
        case FIELD_joinIndex: return pp->getJoinIndex();
        case FIELD_newFormation: return pp->getNewFormation(i);
        default: throw omnetpp::cRuntimeError("Cannot return field %d of class 'CarlaManeuverMessage' as cValue -- field index out of range?", field);
    }
}

void CarlaManeuverMessageDescriptor::setFieldValue(omnetpp::any_ptr object, int field, int i, const omnetpp::cValue& value) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount()){
            base->setFieldValue(object, field, i, value);
            return;
        }
        field -= base->getFieldCount();
    }
    CarlaManeuverMessage *pp = omnetpp::fromAnyPtr<CarlaManeuverMessage>(object); (void)pp;
    switch (field) {
        case FIELD_msgType: pp->setMsgType(omnetpp::checked_int_cast<int>(value.intValue())); break;
        case FIELD_senderId: pp->setSenderId(omnetpp::checked_int_cast<int>(value.intValue())); break;
        case FIELD_platoonId: pp->setPlatoonId(omnetpp::checked_int_cast<int>(value.intValue())); break;
        case FIELD_leaderId: pp->setLeaderId(omnetpp::checked_int_cast<int>(value.intValue())); break;
        case FIELD_joinerId: pp->setJoinerId(omnetpp::checked_int_cast<int>(value.intValue())); break;
        case FIELD_permitted: pp->setPermitted(value.boolValue()); break;
        case FIELD_platoonSpeed: pp->setPlatoonSpeed(value.doubleValue()); break;
        case FIELD_joinIndex: pp->setJoinIndex(omnetpp::checked_int_cast<int>(value.intValue())); break;
        case FIELD_newFormation: pp->setNewFormation(i,omnetpp::checked_int_cast<int>(value.intValue())); break;
        default: throw omnetpp::cRuntimeError("Cannot set field %d of class 'CarlaManeuverMessage'", field);
    }
}

const char *CarlaManeuverMessageDescriptor::getFieldStructName(int field) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldStructName(field);
        field -= base->getFieldCount();
    }
    switch (field) {
        default: return nullptr;
    };
}

omnetpp::any_ptr CarlaManeuverMessageDescriptor::getFieldStructValuePointer(omnetpp::any_ptr object, int field, int i) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldStructValuePointer(object, field, i);
        field -= base->getFieldCount();
    }
    CarlaManeuverMessage *pp = omnetpp::fromAnyPtr<CarlaManeuverMessage>(object); (void)pp;
    switch (field) {
        default: return omnetpp::any_ptr(nullptr);
    }
}

void CarlaManeuverMessageDescriptor::setFieldStructValuePointer(omnetpp::any_ptr object, int field, int i, omnetpp::any_ptr ptr) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount()){
            base->setFieldStructValuePointer(object, field, i, ptr);
            return;
        }
        field -= base->getFieldCount();
    }
    CarlaManeuverMessage *pp = omnetpp::fromAnyPtr<CarlaManeuverMessage>(object); (void)pp;
    switch (field) {
        default: throw omnetpp::cRuntimeError("Cannot set field %d of class 'CarlaManeuverMessage'", field);
    }
}

namespace omnetpp {

}  // namespace omnetpp

