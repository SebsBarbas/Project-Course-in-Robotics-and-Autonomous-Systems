// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: PointStamped.proto

#define INTERNAL_SUPPRESS_PROTOBUF_FIELD_DEPRECATION
#include "PointStamped.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/port.h>
#include <google/protobuf/stubs/once.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)

namespace gz_geometry_msgs {

namespace {

const ::google::protobuf::Descriptor* PointStamped_descriptor_ = NULL;
const ::google::protobuf::internal::GeneratedMessageReflection*
  PointStamped_reflection_ = NULL;

}  // namespace


void protobuf_AssignDesc_PointStamped_2eproto() GOOGLE_ATTRIBUTE_COLD;
void protobuf_AssignDesc_PointStamped_2eproto() {
  protobuf_AddDesc_PointStamped_2eproto();
  const ::google::protobuf::FileDescriptor* file =
    ::google::protobuf::DescriptorPool::generated_pool()->FindFileByName(
      "PointStamped.proto");
  GOOGLE_CHECK(file != NULL);
  PointStamped_descriptor_ = file->message_type(0);
  static const int PointStamped_offsets_[2] = {
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(PointStamped, header_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(PointStamped, point_),
  };
  PointStamped_reflection_ =
    ::google::protobuf::internal::GeneratedMessageReflection::NewGeneratedMessageReflection(
      PointStamped_descriptor_,
      PointStamped::default_instance_,
      PointStamped_offsets_,
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(PointStamped, _has_bits_[0]),
      -1,
      -1,
      sizeof(PointStamped),
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(PointStamped, _internal_metadata_),
      -1);
}

namespace {

GOOGLE_PROTOBUF_DECLARE_ONCE(protobuf_AssignDescriptors_once_);
inline void protobuf_AssignDescriptorsOnce() {
  ::google::protobuf::GoogleOnceInit(&protobuf_AssignDescriptors_once_,
                 &protobuf_AssignDesc_PointStamped_2eproto);
}

void protobuf_RegisterTypes(const ::std::string&) GOOGLE_ATTRIBUTE_COLD;
void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedMessage(
      PointStamped_descriptor_, &PointStamped::default_instance());
}

}  // namespace

void protobuf_ShutdownFile_PointStamped_2eproto() {
  delete PointStamped::default_instance_;
  delete PointStamped_reflection_;
}

void protobuf_AddDesc_PointStamped_2eproto() GOOGLE_ATTRIBUTE_COLD;
void protobuf_AddDesc_PointStamped_2eproto() {
  static bool already_here = false;
  if (already_here) return;
  already_here = true;
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ::gz_std_msgs::protobuf_AddDesc_Header_2eproto();
  ::gz_geometry_msgs::protobuf_AddDesc_Point_2eproto();
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
    "\n\022PointStamped.proto\022\020gz_geometry_msgs\032\014"
    "Header.proto\032\013Point.proto\"[\n\014PointStampe"
    "d\022#\n\006header\030\001 \002(\0132\023.gz_std_msgs.Header\022&"
    "\n\005point\030\002 \002(\0132\027.gz_geometry_msgs.Point", 158);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "PointStamped.proto", &protobuf_RegisterTypes);
  PointStamped::default_instance_ = new PointStamped();
  PointStamped::default_instance_->InitAsDefaultInstance();
  ::google::protobuf::internal::OnShutdown(&protobuf_ShutdownFile_PointStamped_2eproto);
}

// Force AddDescriptors() to be called at static initialization time.
struct StaticDescriptorInitializer_PointStamped_2eproto {
  StaticDescriptorInitializer_PointStamped_2eproto() {
    protobuf_AddDesc_PointStamped_2eproto();
  }
} static_descriptor_initializer_PointStamped_2eproto_;

// ===================================================================

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int PointStamped::kHeaderFieldNumber;
const int PointStamped::kPointFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

PointStamped::PointStamped()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:gz_geometry_msgs.PointStamped)
}

void PointStamped::InitAsDefaultInstance() {
  header_ = const_cast< ::gz_std_msgs::Header*>(&::gz_std_msgs::Header::default_instance());
  point_ = const_cast< ::gz_geometry_msgs::Point*>(&::gz_geometry_msgs::Point::default_instance());
}

PointStamped::PointStamped(const PointStamped& from)
  : ::google::protobuf::Message(),
    _internal_metadata_(NULL) {
  SharedCtor();
  MergeFrom(from);
  // @@protoc_insertion_point(copy_constructor:gz_geometry_msgs.PointStamped)
}

void PointStamped::SharedCtor() {
  _cached_size_ = 0;
  header_ = NULL;
  point_ = NULL;
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
}

PointStamped::~PointStamped() {
  // @@protoc_insertion_point(destructor:gz_geometry_msgs.PointStamped)
  SharedDtor();
}

void PointStamped::SharedDtor() {
  if (this != default_instance_) {
    delete header_;
    delete point_;
  }
}

void PointStamped::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* PointStamped::descriptor() {
  protobuf_AssignDescriptorsOnce();
  return PointStamped_descriptor_;
}

const PointStamped& PointStamped::default_instance() {
  if (default_instance_ == NULL) protobuf_AddDesc_PointStamped_2eproto();
  return *default_instance_;
}

PointStamped* PointStamped::default_instance_ = NULL;

PointStamped* PointStamped::New(::google::protobuf::Arena* arena) const {
  PointStamped* n = new PointStamped;
  if (arena != NULL) {
    arena->Own(n);
  }
  return n;
}

void PointStamped::Clear() {
// @@protoc_insertion_point(message_clear_start:gz_geometry_msgs.PointStamped)
  if (_has_bits_[0 / 32] & 3u) {
    if (has_header()) {
      if (header_ != NULL) header_->::gz_std_msgs::Header::Clear();
    }
    if (has_point()) {
      if (point_ != NULL) point_->::gz_geometry_msgs::Point::Clear();
    }
  }
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
  if (_internal_metadata_.have_unknown_fields()) {
    mutable_unknown_fields()->Clear();
  }
}

bool PointStamped::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:gz_geometry_msgs.PointStamped)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoff(127);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // required .gz_std_msgs.Header header = 1;
      case 1: {
        if (tag == 10) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessageNoVirtual(
               input, mutable_header()));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(18)) goto parse_point;
        break;
      }

      // required .gz_geometry_msgs.Point point = 2;
      case 2: {
        if (tag == 18) {
         parse_point:
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessageNoVirtual(
               input, mutable_point()));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectAtEnd()) goto success;
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0 ||
            ::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_END_GROUP) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:gz_geometry_msgs.PointStamped)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:gz_geometry_msgs.PointStamped)
  return false;
#undef DO_
}

void PointStamped::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:gz_geometry_msgs.PointStamped)
  // required .gz_std_msgs.Header header = 1;
  if (has_header()) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      1, *this->header_, output);
  }

  // required .gz_geometry_msgs.Point point = 2;
  if (has_point()) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      2, *this->point_, output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:gz_geometry_msgs.PointStamped)
}

::google::protobuf::uint8* PointStamped::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  // @@protoc_insertion_point(serialize_to_array_start:gz_geometry_msgs.PointStamped)
  // required .gz_std_msgs.Header header = 1;
  if (has_header()) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageNoVirtualToArray(
        1, *this->header_, false, target);
  }

  // required .gz_geometry_msgs.Point point = 2;
  if (has_point()) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageNoVirtualToArray(
        2, *this->point_, false, target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:gz_geometry_msgs.PointStamped)
  return target;
}

int PointStamped::RequiredFieldsByteSizeFallback() const {
// @@protoc_insertion_point(required_fields_byte_size_fallback_start:gz_geometry_msgs.PointStamped)
  int total_size = 0;

  if (has_header()) {
    // required .gz_std_msgs.Header header = 1;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::MessageSizeNoVirtual(
        *this->header_);
  }

  if (has_point()) {
    // required .gz_geometry_msgs.Point point = 2;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::MessageSizeNoVirtual(
        *this->point_);
  }

  return total_size;
}
int PointStamped::ByteSize() const {
// @@protoc_insertion_point(message_byte_size_start:gz_geometry_msgs.PointStamped)
  int total_size = 0;

  if (((_has_bits_[0] & 0x00000003) ^ 0x00000003) == 0) {  // All required fields are present.
    // required .gz_std_msgs.Header header = 1;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::MessageSizeNoVirtual(
        *this->header_);

    // required .gz_geometry_msgs.Point point = 2;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::MessageSizeNoVirtual(
        *this->point_);

  } else {
    total_size += RequiredFieldsByteSizeFallback();
  }
  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        unknown_fields());
  }
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = total_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void PointStamped::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:gz_geometry_msgs.PointStamped)
  if (GOOGLE_PREDICT_FALSE(&from == this)) {
    ::google::protobuf::internal::MergeFromFail(__FILE__, __LINE__);
  }
  const PointStamped* source = 
      ::google::protobuf::internal::DynamicCastToGenerated<const PointStamped>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:gz_geometry_msgs.PointStamped)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:gz_geometry_msgs.PointStamped)
    MergeFrom(*source);
  }
}

void PointStamped::MergeFrom(const PointStamped& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:gz_geometry_msgs.PointStamped)
  if (GOOGLE_PREDICT_FALSE(&from == this)) {
    ::google::protobuf::internal::MergeFromFail(__FILE__, __LINE__);
  }
  if (from._has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    if (from.has_header()) {
      mutable_header()->::gz_std_msgs::Header::MergeFrom(from.header());
    }
    if (from.has_point()) {
      mutable_point()->::gz_geometry_msgs::Point::MergeFrom(from.point());
    }
  }
  if (from._internal_metadata_.have_unknown_fields()) {
    mutable_unknown_fields()->MergeFrom(from.unknown_fields());
  }
}

void PointStamped::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:gz_geometry_msgs.PointStamped)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void PointStamped::CopyFrom(const PointStamped& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:gz_geometry_msgs.PointStamped)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool PointStamped::IsInitialized() const {
  if ((_has_bits_[0] & 0x00000003) != 0x00000003) return false;

  if (has_header()) {
    if (!this->header_->IsInitialized()) return false;
  }
  if (has_point()) {
    if (!this->point_->IsInitialized()) return false;
  }
  return true;
}

void PointStamped::Swap(PointStamped* other) {
  if (other == this) return;
  InternalSwap(other);
}
void PointStamped::InternalSwap(PointStamped* other) {
  std::swap(header_, other->header_);
  std::swap(point_, other->point_);
  std::swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
  std::swap(_cached_size_, other->_cached_size_);
}

::google::protobuf::Metadata PointStamped::GetMetadata() const {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::Metadata metadata;
  metadata.descriptor = PointStamped_descriptor_;
  metadata.reflection = PointStamped_reflection_;
  return metadata;
}

#if PROTOBUF_INLINE_NOT_IN_HEADERS
// PointStamped

// required .gz_std_msgs.Header header = 1;
bool PointStamped::has_header() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
void PointStamped::set_has_header() {
  _has_bits_[0] |= 0x00000001u;
}
void PointStamped::clear_has_header() {
  _has_bits_[0] &= ~0x00000001u;
}
void PointStamped::clear_header() {
  if (header_ != NULL) header_->::gz_std_msgs::Header::Clear();
  clear_has_header();
}
const ::gz_std_msgs::Header& PointStamped::header() const {
  // @@protoc_insertion_point(field_get:gz_geometry_msgs.PointStamped.header)
  return header_ != NULL ? *header_ : *default_instance_->header_;
}
::gz_std_msgs::Header* PointStamped::mutable_header() {
  set_has_header();
  if (header_ == NULL) {
    header_ = new ::gz_std_msgs::Header;
  }
  // @@protoc_insertion_point(field_mutable:gz_geometry_msgs.PointStamped.header)
  return header_;
}
::gz_std_msgs::Header* PointStamped::release_header() {
  // @@protoc_insertion_point(field_release:gz_geometry_msgs.PointStamped.header)
  clear_has_header();
  ::gz_std_msgs::Header* temp = header_;
  header_ = NULL;
  return temp;
}
void PointStamped::set_allocated_header(::gz_std_msgs::Header* header) {
  delete header_;
  header_ = header;
  if (header) {
    set_has_header();
  } else {
    clear_has_header();
  }
  // @@protoc_insertion_point(field_set_allocated:gz_geometry_msgs.PointStamped.header)
}

// required .gz_geometry_msgs.Point point = 2;
bool PointStamped::has_point() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
void PointStamped::set_has_point() {
  _has_bits_[0] |= 0x00000002u;
}
void PointStamped::clear_has_point() {
  _has_bits_[0] &= ~0x00000002u;
}
void PointStamped::clear_point() {
  if (point_ != NULL) point_->::gz_geometry_msgs::Point::Clear();
  clear_has_point();
}
const ::gz_geometry_msgs::Point& PointStamped::point() const {
  // @@protoc_insertion_point(field_get:gz_geometry_msgs.PointStamped.point)
  return point_ != NULL ? *point_ : *default_instance_->point_;
}
::gz_geometry_msgs::Point* PointStamped::mutable_point() {
  set_has_point();
  if (point_ == NULL) {
    point_ = new ::gz_geometry_msgs::Point;
  }
  // @@protoc_insertion_point(field_mutable:gz_geometry_msgs.PointStamped.point)
  return point_;
}
::gz_geometry_msgs::Point* PointStamped::release_point() {
  // @@protoc_insertion_point(field_release:gz_geometry_msgs.PointStamped.point)
  clear_has_point();
  ::gz_geometry_msgs::Point* temp = point_;
  point_ = NULL;
  return temp;
}
void PointStamped::set_allocated_point(::gz_geometry_msgs::Point* point) {
  delete point_;
  point_ = point;
  if (point) {
    set_has_point();
  } else {
    clear_has_point();
  }
  // @@protoc_insertion_point(field_set_allocated:gz_geometry_msgs.PointStamped.point)
}

#endif  // PROTOBUF_INLINE_NOT_IN_HEADERS

// @@protoc_insertion_point(namespace_scope)

}  // namespace gz_geometry_msgs

// @@protoc_insertion_point(global_scope)
