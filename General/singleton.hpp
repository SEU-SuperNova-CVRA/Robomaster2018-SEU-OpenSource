#pragma once
#ifndef SCH_SINGLETON
#define SCH_SINGLETON

//Meyer's Singleton, Simple and fine
/****************************WARNING****************************/
//it can only work in C++11 and later
/****************************WARNING****************************/
//Because of C++11 Standard 6.7:
/*
   If control enters the declaration concurrently while the variable is being initialized, 
   the concurrent execution shall wait for completion of the initialization.
*/
//It will not leak
//play fine with it!
template <typename T>
class Singleton
{
public:
	
	static T& getInstance()
	{
		static T value;
		return value;
	}

protected:
	Singleton() {};
	~Singleton() {};
};
#endif // !1