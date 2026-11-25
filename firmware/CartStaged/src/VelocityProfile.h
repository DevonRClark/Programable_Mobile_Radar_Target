#pragma once
#include "CartCommon.h"
namespace control {
// Fixed staging/active tables; control-task owned. CRC work only while stopped.
struct VelocityProfile {
 struct Point { float x=0,v=0; };
 static_assert(sizeof(Point)==8,"Profile CRC requires8byte IEEE-float points");
 Point active[cart::PROFILE_MAX_POINTS]{},staging[cart::PROFILE_MAX_POINTS]{};
 uint8_t count=0,expected=0,received=0;
 bool uploading=false,failed=false;
 uint32_t crc=0;
 float duration=0,floor=cart::PROFILE_SPEED_FLOOR_MPS,staging_floor=cart::PROFILE_SPEED_FLOOR_MPS,launch=cart::PROFILE_SPEED_FLOOR_MPS,staging_launch=cart::PROFILE_SPEED_FLOOR_MPS;
 void clear(){count=expected=received=0;uploading=failed=false;crc=0;duration=0;floor=staging_floor=launch=staging_launch=cart::PROFILE_SPEED_FLOOR_MPS;}
 static uint32_t checksum(const Point *points,unsigned n){
  const auto *p=reinterpret_cast<const uint8_t*>(points);uint32_t c=0xffffffff;
  for(unsigned i=0;i<n*sizeof(Point);++i){c^=p[i];for(unsigned bit=0;bit<8;++bit)c=(c>>1)^(0xedb88320u & (0u-(c&1u)));}
  return c^0xffffffff;
 }
 bool begin(uint32_t n,float minimum=cart::PROFILE_SPEED_FLOOR_MPS,float launch_target=-1){
  if(launch_target==-1)launch_target=minimum;
  if(!isfinite(launch_target) || launch_target<minimum || launch_target>cart::MAX_SPEED_MPS || !isfinite(minimum) || minimum<.1f || minimum>cart::MAX_SPEED_MPS || n<2 || n>cart::PROFILE_MAX_POINTS)return false;
  staging_launch=launch_target;staging_floor=minimum;expected=static_cast<uint8_t>(n);received=0;uploading=true;failed=false;return true;
 }
 bool point(uint32_t index,float x,float v){
  if(!uploading || failed || index>=expected || !isfinite(x)||!isfinite(v) || x<0 || x>cart::PROFILE_MAX_DISTANCE_M || v<0 || v>cart::MAX_SPEED_MPS)return false;
  if(index<received){if(staging[index].x==x && staging[index].v==v)return true;failed=true;return false;}
  if(index!=received){failed=true;return false;}
  staging[index]={x,v};++received;return true;
 }
 bool commit(uint32_t wanted){
  if(!uploading || failed || received!=expected || checksum(staging,received)!=wanted)return false;
  if(staging[0].x!=0 || staging[0].v!=0 || staging[received-1].v!=0 || staging[received-1].x<.5f)return false;
  float seconds=0;
  for(unsigned i=1;i<received;++i){
   const auto &a=staging[i-1],&b=staging[i];const float dx=b.x-a.x;
   if(dx<=0 || (i+1<received && b.v<=0) || a.v+b.v<=0)return false;
   const float acceleration=(b.v*b.v-a.v*a.v)/(2*dx);
   if(acceleration>cart::PROFILE_MAX_ACCEL_MPS2+.00001f || acceleration < -cart::PROFILE_MAX_DECEL_MPS2-.00001f)return false;
   seconds+=2*dx/(a.v+b.v);
  }
  float maximum=0;for(unsigned i=0;i<received;++i)maximum=fmaxf(maximum,staging[i].v);
  if(staging_launch>maximum || staging_floor>maximum || !isfinite(seconds) || seconds+4>cart::APPROACH_MAX_RUNTIME_US/1000000.f)return false;
  memcpy(active,staging,received*sizeof(Point));count=received;crc=wanted;duration=seconds;floor=staging_floor;launch=staging_launch;uploading=false;return true;
 }
 bool ready()const{return count>=2 && !uploading;}
 float length()const{return count ? active[count-1].x:0;}
 float peak()const{float v=0;for(unsigned i=0;i<count;++i)v=fmaxf(v,active[i].v);return v;}
 float speedAt(float x,bool &descending,unsigned &segment)const{
  descending=false;segment=0;if(count<2 || x>=length())return 0;
  x=fmaxf(x,0);
  while(segment+2<count && x>active[segment+1].x)++segment;
  const auto &a=active[segment],&b=active[segment+1];descending=b.v<a.v;
  const float f=cart::clampf((x-a.x)/(b.x-a.x),0,1);
  return sqrtf(fmaxf(0,a.v*a.v+f*(b.v*b.v-a.v*a.v)));
 }
};
} // namespace control
