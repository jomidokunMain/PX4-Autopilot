#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/wheel_encoders.h>

class MavlinkStreamEncoder : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamEncoder(mavlink); }

	static constexpr const char *get_name_static() { return "ENCODER"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_WHEEL_DISTANCE; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		// if (_wheel_encoders_sub.advertised()) {
		// 	return MAVLINK_MSG_ID_WHEEL_DISTANCE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
		// }

		// return 0;

		return (MAVLINK_MSG_ID_WHEEL_DISTANCE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES);
	}
private:
	explicit MavlinkStreamEncoder(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	//uORB::Subscription _wheel_encoders_sub{ORB_ID(wheel_encoders)};
	uORB::SubscriptionMultiArray<wheel_encoders_s> _wheel_encoders_subs{ORB_ID::wheel_encoders};

	bool send() override
	{
		bool updated = false;

		mavlink_wheel_distance_t msg{};

		int count = 0;
		for(int i = 0; i < _wheel_encoders_subs.size(); i++) {
			wheel_encoders_s wheel_encoders;

			if (_wheel_encoders_subs[i].update(&wheel_encoders)) {
				msg.time_usec = wheel_encoders.timestamp;
				//TO DO: Convert encoder pulses to distance
				msg.distance[i] = wheel_encoders.encoder_position;
				count += 1;
			}
		}

		if(count > 0) {
			msg.count = count;
			mavlink_msg_wheel_distance_send_struct(_mavlink->get_channel(), &msg);

			updated = true;
		}

		return updated;


		// wheel_encoders_s wheel_encoders;

		// if(_wheel_encoders_sub.update(&wheel_encoders)) {
		// 	mavlink_wheel_distance_t msg{};

		// 	msg.time_usec = wheel_encoders.timestamp;
		// 	msg.count = 1;
		// 	msg.distance[0] = wheel_encoders.encoder_position;

		// 	mavlink_msg_wheel_distance_send_struct(_mavlink->get_channel(), &msg);

		// 	return true;
		// }

		// return false;
	}
};

#endif // ENCODER_HPP
