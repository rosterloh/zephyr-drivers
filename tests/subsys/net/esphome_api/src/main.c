/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <zephyr/ztest.h>
#include <zephyr/net/esphome_api.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include <api.pb.h>

ZTEST(esphome_api_proto, test_hello_response_encode)
{
	HelloResponse resp = {
		.api_version_major = ESPHOME_API_VERSION_MAJOR,
		.api_version_minor = ESPHOME_API_VERSION_MINOR,
		.name = "x",
	};
	/* major=1, minor=19, name="x"; empty server_info omitted (proto3) */
	static const uint8_t expected[] = {0x08, 0x01, 0x10, 0x13, 0x22, 0x01, 'x'};
	uint8_t buf[HelloResponse_size];
	pb_ostream_t os = pb_ostream_from_buffer(buf, sizeof(buf));

	zassert_true(pb_encode(&os, HelloResponse_fields, &resp));
	zassert_equal(os.bytes_written, sizeof(expected));
	zassert_mem_equal(buf, expected, sizeof(expected));
}

ZTEST(esphome_api_proto, test_hello_request_decode)
{
	static const uint8_t wire[] = {0x0a, 0x02, 'H', 'A', 0x10, 0x01, 0x18, 0x13};
	HelloRequest req = HelloRequest_init_zero;
	pb_istream_t is = pb_istream_from_buffer(wire, sizeof(wire));

	zassert_true(pb_decode(&is, HelloRequest_fields, &req));
	zassert_str_equal(req.client_info, "HA");
	zassert_equal(req.api_version_major, 1);
	zassert_equal(req.api_version_minor, 19);
}

ZTEST(esphome_api_proto, test_client_info_too_long)
{
	uint8_t wire[2 + sizeof(((HelloRequest *)0)->client_info)];
	HelloRequest req;
	pb_istream_t is;

	/* one byte more than fits with the NUL */
	wire[0] = 0x0a;
	wire[1] = sizeof(wire) - 2;
	memset(&wire[2], 'a', sizeof(wire) - 2);
	is = pb_istream_from_buffer(wire, sizeof(wire));
	zassert_false(pb_decode(&is, HelloRequest_fields, &req));
}

ZTEST(esphome_api_proto, test_switch_command_decode)
{
	static const uint8_t wire[] = {0x0d, 0x78, 0x56, 0x34, 0x12, 0x10, 0x01};
	SwitchCommandRequest cmd = SwitchCommandRequest_init_zero;
	pb_istream_t is = pb_istream_from_buffer(wire, sizeof(wire));

	zassert_true(pb_decode(&is, SwitchCommandRequest_fields, &cmd));
	zassert_equal(cmd.key, 0x12345678);
	zassert_true(cmd.state);
}

ZTEST(esphome_api_proto, test_msgids)
{
	zassert_equal(HelloRequest_msgid, 1);
	zassert_equal(HelloResponse_msgid, 2);
	zassert_equal(ListEntitiesDoneResponse_msgid, 19);
	zassert_equal(ButtonCommandRequest_msgid, 62);
}

ZTEST_SUITE(esphome_api_proto, NULL, NULL, NULL, NULL, NULL);
