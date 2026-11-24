#pragma once
#include <stdint.h>
namespace actuator_current {
constexpr uint32_t TARGET_RAW = 675;
constexpr uint32_t MIN_ACCEPTED_RAW = 670;
class Guard {
public:
    enum Reason { NONE, CANCELLED, INVALID_CHANNEL, READ_FAILED, NONZERO_PWM,
                  INVALID_LIMIT, WRITE_FAILED, VERIFY_FAILED, LIMIT_CHANGED,
                  RETURN_LIMIT_CHANGED };
    void beginCommand(){prepared_=0;if(!failed_)reason_=NONE;}
    void acknowledgeStop(){failed_=false;prepared_=0;reason_=NONE;}
    bool failed()const{return failed_;}
    bool cancelled()const{return reason_==CANCELLED;}
    bool observed()const{return observed_;}
    uint32_t lastLimit()const{return last_limit_;}
    uint32_t lastReturnLimit()const{return last_return_limit_;}
    const char* reason()const{
        switch(reason_){
            case CANCELLED:return "USB input/disconnect";
            case INVALID_CHANNEL:return "invalid channel";
            case READ_FAILED:return "current-limit read failed";
            case NONZERO_PWM:return "zero PWM not confirmed";
            case INVALID_LIMIT:return "initial limit invalid";
            case WRITE_FAILED:return "limit write not acknowledged";
            case VERIFY_FAILED:return "limit readback outside 6.70-6.75 A";
            case LIMIT_CHANGED:return "limit changed during command";
            case RETURN_LIMIT_CHANGED:return "regen/min raw limit changed";
            default:return "none";
        }
    }
    static bool matches(uint32_t value){return value>=MIN_ACCEPTED_RAW && value<=TARGET_RAW;}
    template<class IO> bool allow(uint8_t channel,IO&io){
        if(failed_)return false;
        reason_=NONE;observed_=false;
        if(io.cancelled())return cancel();
        if(channel<1||channel>2)return fail(INVALID_CHANNEL);
        uint32_t value=0,return_limit=0;
        // Verify BOTH fields before EVERY nonzero packet. The second word is
        // reserved on old firmware and a regen/min limit on newer firmware:
        // preserve its exact bits without guessing units or a safe new value.
        if(!io.readLimit(channel,value,return_limit))return fail(READ_FAILED);
        last_limit_=value;last_return_limit_=return_limit;observed_=true;
        if(io.cancelled())return cancel();
        const uint8_t bit=uint8_t(1U<<(channel-1));
        if(prepared_&bit){
            if(!matches(value))return fail(LIMIT_CHANGED);
            return return_limit==prepared_return_limit_[channel-1]?true:fail(RETURN_LIMIT_CHANGED);
        }
        // Never rewrite settings on a controller with unconfirmed zero PWM.
        if(!io.zeroPwm())return fail(NONZERO_PWM);
        if(io.cancelled())return cancel();
        if(!matches(value)){
            if(value==0 || value>1500)return fail(INVALID_LIMIT);
            const uint32_t preserved_return_limit=return_limit;
            if(!io.writeLimit(channel,TARGET_RAW,preserved_return_limit))return fail(WRITE_FAILED);
            if(io.cancelled())return cancel();
            if(!io.readLimit(channel,value,return_limit))return fail(READ_FAILED);
            last_limit_=value;last_return_limit_=return_limit;
            if(io.cancelled())return cancel();
            if(return_limit!=preserved_return_limit)return fail(RETURN_LIMIT_CHANGED);
            if(!matches(value))return fail(VERIFY_FAILED);
        }
        prepared_return_limit_[channel-1]=return_limit;
        prepared_|=bit;return true;
    }
private:
    uint8_t prepared_=0;bool failed_=false,observed_=false;
    uint32_t last_limit_=0,last_return_limit_=0;
    uint32_t prepared_return_limit_[2]={};
    Reason reason_=NONE;
    bool fail(Reason reason){failed_=true;prepared_=0;reason_=reason;return false;}
    // A pending operator command blocks the packet and ends the caller's move.
    // It is not evidence that the controller's current setting is incorrect.
    bool cancel(){prepared_=0;reason_=CANCELLED;return false;}
};
}
