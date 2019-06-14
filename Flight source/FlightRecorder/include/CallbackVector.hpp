/*
	CallbackVector.hpp
	
	Class for handling a vector of callbacks.
	
	2019-06-13	JDW	Created.
*/

#ifndef __CALLBACKVECTOR_HPP__
#define __CALLBACKVECTOR_HPP__

#include <vector>
#include <functional>

using namespace std;

template <typename ArgType>
class CallbackVector {
public:
	/**
	 * Constructor. 
	 */
	CallbackVector() { 
		
	}
	virtual ~CallbackVector() {
		
	}
	
	// Callbacks shall return void and take one argument
	typedef function<void(ArgType)> callbackFtnType;
	
	/**
	 * Register a callback lacking a this pointer.
	 * That is, a static method, or a function that is not a class method, or a lambda.
	 * @param callback function returning void and taking one argument
	 */
	void registerStaticCallback(const callbackFtnType &callback);
	/**
	 * Register a callback that is a non-static member function
	 * @tparam OwnerClass the type of param callbackOwner
	 * @param callback pointer to a member function returning void and taking one argument
	 * @param callbackOwner the `this` pointer for the member function
	 */
	template<typename OwnerClass>
	void registerMemberCallback(void (OwnerClass::*callback)(ArgType), OwnerClass * callbackOwner);
	
	/**
	 * Call all callbacks registered to this callback vector.
	 * @param arg the argument to pass to each callback
	 */
	void fireCallbacks(ArgType arg);
	
private:
	vector<callbackFtnType> callbackVector;
};

#endif // __CALLBACKVECTOR_HPP__

