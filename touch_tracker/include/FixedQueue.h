//
//  Created by Robert Xiao on April 8, 2015.
//
#pragma once

template < typename T, int N >
struct fixedqueue
{
protected:
	T vals[ N ];
	int rpos, wpos, count;

public:
	fixedqueue() : rpos( 0 ), wpos( 0 ), count( 0 )
	{}

	int size() const
	{
		return count;
	}

	bool empty() const
	{
		return count == 0;
	}

	T pop()
	{
		assert( count > 0 );
		T ret = vals[ rpos ];
		rpos = ( rpos + 1 ) % N;
		--count;
		return ret;
	}

	void push( const T& val )
	{
		assert( count < N );
		vals[ wpos ] = val;
		wpos = ( wpos + 1 ) % N;
		++count;
	}
};