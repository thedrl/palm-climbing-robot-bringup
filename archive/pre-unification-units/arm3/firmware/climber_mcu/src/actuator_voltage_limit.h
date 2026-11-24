#pragma once
#include <stdint.h>
namespace actuator_voltage {
constexpr uint16_t TARGET_MAX_RAW=252;
constexpr uint16_t MIN_ACCEPTED_MAX_RAW=251;
constexpr uint16_t BATTERY_MODE_MASK=0x001c;
struct Limits { uint16_t minimum=0,maximum=0;uint8_t offset=0; };
class Guard {
public:
    enum Reason { NONE,CANCELLED,READ_FAILED,BATTERY_PRESET,INVALID_LIMITS,
                  NONZERO_PWM,WRITE_FAILED,VERIFY_FAILED,INVALID_RESTORE };
    void beginCommand(){prepared_=false;if(!failed_)reason_=NONE;}
    void acknowledgeStop(){prepared_=false;failed_=false;reason_=NONE;}
    bool failed()const{return failed_;}
    bool cancelled()const{return reason_==CANCELLED;}
    bool prepared()const{return prepared_;}
    uint16_t verifiedMaximum()const{return prepared_?verified_maximum_:0;}
    uint16_t verifiedMinimum()const{return prepared_?verified_minimum_:0;}
    static bool matches(uint16_t maximum){return maximum>=MIN_ACCEPTED_MAX_RAW&&maximum<=TARGET_MAX_RAW;}
    bool readFailure(){return fail(READ_FAILED);}
    bool cancelPreparation(){return cancel();}
    const char* reason()const{
        switch(reason_){
            case CANCELLED:return "USB input/disconnect";
            case READ_FAILED:return "voltage/configuration read failed";
            case BATTERY_PRESET:return "battery preset enabled; user voltage settings required";
            case INVALID_LIMITS:return "invalid voltage limits or minimum >=25.1 V";
            case NONZERO_PWM:return "zero PWM not confirmed";
            case WRITE_FAILED:return "voltage write not acknowledged";
            case VERIFY_FAILED:return "voltage/configuration readback changed";
            case INVALID_RESTORE:return "restore requires current minimum +1 (raw), <=250, and verified maximum";
            default:return "none";
        }
    }
    template<class IO> bool allow(IO&io,bool require_idle=false){
        return prepare(io,require_idle,false,0);
    }
    // Explicit one-count restoration of an externally recorded prior minimum.
    // Never chooses a baseline, saves NVM, or sends a motor command.
    template<class IO> bool restoreMinimum(IO&io,uint16_t target){
        return prepare(io,true,true,target);
    }
private:
    template<class IO> bool prepare(IO&io,bool require_idle,bool restore,uint16_t target){
        if(failed_)return false;
        reason_=NONE;
        if(io.cancelled())return cancel();
        // Prepare once per command, never rewrite settings during a refresh.
        if(prepared_&&!require_idle&&!restore)return true;
        prepared_=false;
        uint16_t config=0;Limits original;
        if(!io.readConfig(config))return fail(READ_FAILED);
        if(io.cancelled())return cancel();
        if(config&BATTERY_MODE_MASK)return fail(BATTERY_PRESET);
        if(!io.readLimits(original))return fail(READ_FAILED);
        if(io.cancelled())return cancel();
        // Reserve one count for a bounded compensation write if needed.
        if(original.minimum>=MIN_ACCEPTED_MAX_RAW||original.maximum<=original.minimum)
            return fail(INVALID_LIMITS);
        if(restore && (target>250 || target!=original.minimum+1 || !matches(original.maximum)))
            return fail(INVALID_RESTORE);
        const bool needs_write=restore||!matches(original.maximum);
        Limits requested=original;
        if(restore)requested.minimum=target;
        uint16_t verified_maximum=original.maximum;
        if(require_idle||needs_write){
            if(!io.zeroPwm())return fail(NONZERO_PWM);
            if(io.cancelled())return cancel();
        }
        if(needs_write){
            requested.maximum=TARGET_MAX_RAW;
            const uint16_t exact_minimum=requested.minimum;
            // A single compensating retry is allowed only for the observed
            // one-count lower minimum, with every other field still valid.
            for(unsigned attempt=0;attempt<2;++attempt){
                if(!io.writeLimits(requested))return fail(WRITE_FAILED);
                if(io.cancelled())return cancel();
                Limits observed;uint16_t observed_config=0;
                if(!io.readLimits(observed))return fail(READ_FAILED);
                if(io.cancelled())return cancel();
                if(!io.readConfig(observed_config))return fail(READ_FAILED);
                if(io.cancelled())return cancel();
                if(!matches(observed.maximum)||observed.maximum<=observed.minimum||
                   observed.offset!=original.offset||observed_config!=config)
                    return fail(VERIFY_FAILED);
                if(observed.minimum==exact_minimum){
                    verified_maximum=observed.maximum;break;
                }
                if(attempt!=0 || exact_minimum==0 || observed.minimum!=exact_minimum-1)
                    return fail(VERIFY_FAILED);
                if(!io.zeroPwm())return fail(NONZERO_PWM);
                if(io.cancelled())return cancel();
                requested.minimum=exact_minimum+1; // <=251; maximum stays252
            }
            verified_minimum_=exact_minimum;
        }else verified_minimum_=original.minimum;
        verified_maximum_=verified_maximum;
        prepared_=true;return true;
    }
    bool prepared_=false,failed_=false;Reason reason_=NONE;
    uint16_t verified_maximum_=0,verified_minimum_=0;
    bool fail(Reason reason){prepared_=false;failed_=true;reason_=reason;return false;}
    bool cancel(){prepared_=false;reason_=CANCELLED;return false;}
};
}
