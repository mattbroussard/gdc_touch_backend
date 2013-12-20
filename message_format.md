
# types

## msg_meta

		{
			src //string ID referring to message source
			dest //string ID referring to message destination
			id //int uniquely identifying this message within the context of the one-way exchange (not necessarily unique between clients, or between client and server)
		}

## msg

		{
			meta //object of type msg_meta with message metadata
			call //string specifying what call is being made
			payload //object with whatever properties required for call
		}

## receipt

		{
			receipt //set to true to specify this is a receipt, rather than an actual message
			meta //object of type msg_meta with message metadata -- for receipt to be accepted, all 3 meta fields must match what was expected
		}

# implementation

Both clients and server are expected to store a list of recently sent messages and received msg_meta objects. Sent messages can be resent if a receipt is not received, and the received list is used to prevent accidentally processing the same message twice.
