"use client";

import Link from "next/link";
import { useSession, signOut } from "next-auth/react";
import { Wrench, Menu, X } from "lucide-react";
import { useState } from "react";

export function Navbar() {
  const { data: session } = useSession();
  const [open, setOpen] = useState(false);
  const user = session?.user as { role?: string } | undefined;

  const dashboardHref =
    user?.role === "PROVIDER" ? "/provider/dashboard" : "/customer/dashboard";

  return (
    <nav className="bg-white border-b border-gray-200 sticky top-0 z-50">
      <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
        <div className="flex justify-between h-16">
          <div className="flex items-center gap-2">
            <Link href="/" className="flex items-center gap-2 font-bold text-xl text-blue-600">
              <Wrench className="h-6 w-6" />
              FixUsNow
            </Link>
          </div>

          <div className="hidden md:flex items-center gap-6">
            <Link href="/tasks" className="text-gray-600 hover:text-blue-600 font-medium">
              Browse Tasks
            </Link>
            {session ? (
              <>
                <Link href={dashboardHref} className="text-gray-600 hover:text-blue-600 font-medium">
                  Dashboard
                </Link>
                {user?.role === "CUSTOMER" && (
                  <Link
                    href="/customer/post-task"
                    className="bg-blue-600 text-white px-4 py-2 rounded-lg font-medium hover:bg-blue-700"
                  >
                    Post a Task
                  </Link>
                )}
                <button
                  onClick={() => signOut({ callbackUrl: "/" })}
                  className="text-gray-600 hover:text-red-600 font-medium"
                >
                  Sign Out
                </button>
              </>
            ) : (
              <>
                <Link href="/login" className="text-gray-600 hover:text-blue-600 font-medium">
                  Sign In
                </Link>
                <Link
                  href="/signup"
                  className="bg-blue-600 text-white px-4 py-2 rounded-lg font-medium hover:bg-blue-700"
                >
                  Sign Up
                </Link>
              </>
            )}
          </div>

          <button className="md:hidden p-2" onClick={() => setOpen(!open)}>
            {open ? <X /> : <Menu />}
          </button>
        </div>
      </div>

      {open && (
        <div className="md:hidden border-t border-gray-200 bg-white px-4 pb-4 space-y-3">
          <Link href="/tasks" className="block py-2 text-gray-700">Browse Tasks</Link>
          {session ? (
            <>
              <Link href={dashboardHref} className="block py-2 text-gray-700">Dashboard</Link>
              {user?.role === "CUSTOMER" && (
                <Link href="/customer/post-task" className="block py-2 text-blue-600 font-medium">
                  Post a Task
                </Link>
              )}
              <button
                onClick={() => signOut({ callbackUrl: "/" })}
                className="block py-2 text-red-600"
              >
                Sign Out
              </button>
            </>
          ) : (
            <>
              <Link href="/login" className="block py-2 text-gray-700">Sign In</Link>
              <Link href="/signup" className="block py-2 text-blue-600 font-medium">Sign Up</Link>
            </>
          )}
        </div>
      )}
    </nav>
  );
}
