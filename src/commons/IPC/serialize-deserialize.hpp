#pragma once
#ifndef SERIALIZE_DESERIALIZE_HPP
#define SERIALIZE_DESERIALIZE_HPP

/* Functions for serialising and deserialing the data using Boost
for sending over SHM */

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/vector.hpp>

template <typename T>
class Serialize
{
public:
	friend class boost::serialization::access;

	void serialize(T genericData, std::stringstream &ss)
	{
		boost::archive::text_oarchive oa(ss);
		oa << genericData;
		// std::string temp = ss.str();
	}

	T deserialize(std::stringstream &ss, T data)
	{
		boost::archive::text_iarchive ia(ss);
		ia >> data;
		return data;
	}
};

#endif